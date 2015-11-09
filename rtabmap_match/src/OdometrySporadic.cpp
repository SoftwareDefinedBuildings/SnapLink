#include "rtabmap/core/Odometry.h"
#include "rtabmap/core/Memory.h"
#include "rtabmap/core/Signature.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_features.h"
#include "rtabmap/core/EpipolarGeometry.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/UStl.h"
#include "rtabmap/core/VWDictionary.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <pcl/common/centroid.h>

#include "OdometrySporadic.h"
#include <list>
#include "rtabmap/core/Features2d.h"
#include "rtabmap/core/Graph.h"
#include <rtabmap/utilite/UMath.h>
#include "MemoryLoc.h"


namespace rtabmap {

OdometrySporadic::OdometrySporadic(const std::string dbPath, const rtabmap::ParametersMap & parameters) :
Odometry(parameters),
dbPath_(dbPath)
{
    UDEBUG("");
    // Setup memory
    memoryParameters_.insert(ParametersPair(Parameters::kMemRehearsalSimilarity(), "1.0")); // desactivate rehearsal
    memoryParameters_.insert(ParametersPair(Parameters::kMemBinDataKept(), "false"));
    memoryParameters_.insert(ParametersPair(Parameters::kMemImageKept(), "true"));
    memoryParameters_.insert(ParametersPair(Parameters::kMemSTMSize(), "0"));
    memoryParameters_.insert(ParametersPair(Parameters::kMemNotLinkedNodesKept(), "false"));
    memoryParameters_.insert(ParametersPair(Parameters::kKpTfIdfLikelihoodUsed(), "false"));
    int featureType = Feature2D::kFeatureSurf;
    memoryParameters_.insert(ParametersPair(Parameters::kKpDetectorStrategy(), uNumber2Str(featureType)));

    // parameters that makes memory do PnP localization for RGB images
    memoryParameters_.insert(ParametersPair(Parameters::kLccBowEstimationType(), "1")); // 1 is PnP
    memoryParameters_.insert(ParametersPair(Parameters::kMemIncrementalMemory(), "false")); 
    memoryParameters_.insert(ParametersPair(Parameters::kLccBowMinInliers(), "20")); 

    memory_ = new Memory();
    if(!memory_ || !memory_->init(dbPath_, false, memoryParameters_))
    {
        UERROR("Error initializing the memory for OdometrySporadic.");
    }
}

OdometrySporadic::~OdometrySporadic()
{
    UDEBUG("");
    delete memory_;
}

void OdometrySporadic::reset(const Transform & initialPose)
{
    UDEBUG("");
    Odometry::reset(initialPose);
}

Transform OdometrySporadic::computeTransform(const SensorData & data, OdometryInfo *info)
{
    UDEBUG("");
    Transform output;

    if(data.imageRaw().empty())
    {
        UERROR("Image empty! Cannot compute odometry...");
        return output;
    }

    if(!(((data.cameraModels().size() == 1 && data.cameraModels()[0].isValid()) || data.stereoCameraModel().isValid())))
    {
        UERROR("Odometry cannot be done without calibration or on multi-camera!");
        return output;
    }

    if(data.imageRaw().channels() != 1)
    {
        UERROR("OdometrySporadic can only take gray images!");
        return output;
    }

    if(info == 0)
    {
        UERROR("info has to be not NULL in OdometrySporadic");
        return output;
    }

    const CameraModel & cameraModel = data.stereoCameraModel().isValid()?data.stereoCameraModel().left():data.cameraModels()[0];

    UTimer timer;

    if(memory_->getWorkingMem().size() >= 1)
    {
        //PnP
        UDEBUG("PnP");

        if(this->isInfoDataFilled() && info)
        {
            info->type = 0; // 0=BOW, 1=Optical Flow, 2=ICP
        }

        // generate kpts
        if(memory_->update(data))
        {
            UDEBUG("");
            const Signature * newS = memory_->getLastWorkingSignature();
            UDEBUG("newWords=%d", (int)newS->getWords().size());
            if((int)newS->getWords().size() > this->getMinInliers())
            {
                std::map<int, float> likelihood;

                ULOGGER_INFO("computing likelihood...");
                std::list<int> signaturesToCompare = uKeysList(memory_->getWorkingMem());
                UDEBUG("signaturesToCompare.size() = %d", signaturesToCompare.size());
                likelihood = memory_->computeLikelihood(newS, signaturesToCompare);
                
                this->adjustLikelihood(likelihood);

                std::vector<int> topIds;
                likelihood.erase(-1);
                int topk_ = 1;
                if(likelihood.size())
                {
                    std::vector< std::pair<int, float> > top(topk_);
                    std::partial_sort_copy(likelihood.begin(),
                                           likelihood.end(),
                                           top.begin(),
                                           top.end(),
                                           compareLikelihood);
                    for(std::vector< std::pair<int, float> >::iterator it = top.begin(); it != top.end(); ++it) {
                        topIds.push_back(it->first);
                    }
                }

                // calculate transform between data and the most similar pose
                ParametersMap customParameters = memoryParameters_; // get BOW LCC parameters
                // override some parameters
                uInsert(customParameters, ParametersPair(Parameters::kMemIncrementalMemory(), "true")); // make sure it is incremental
                uInsert(customParameters, ParametersPair(Parameters::kMemRehearsalSimilarity(), "1.0")); // desactivate rehearsal
                uInsert(customParameters, ParametersPair(Parameters::kMemBinDataKept(), "false"));
                uInsert(customParameters, ParametersPair(Parameters::kMemSTMSize(), "0"));
                uInsert(customParameters, ParametersPair(Parameters::kKpIncrementalDictionary(), "true")); // make sure it is incremental
                uInsert(customParameters, ParametersPair(Parameters::kKpNewWordsComparedTogether(), "false"));
                uInsert(customParameters, ParametersPair(Parameters::kKpNNStrategy(), uNumber2Str(VWDictionary::kNNBruteForce))); // bruteforce
                uInsert(customParameters, ParametersPair(Parameters::kKpNndrRatio(), "0.3")); 
                uInsert(customParameters, ParametersPair(Parameters::kKpDetectorStrategy(), uNumber2Str(Feature2D::kFeatureSurf))); // FAST/BRIEF
                uInsert(customParameters, ParametersPair(Parameters::kKpWordsPerImage(), "1500"));
                uInsert(customParameters, ParametersPair(Parameters::kKpBadSignRatio(), "0"));
                uInsert(customParameters, ParametersPair(Parameters::kKpRoiRatios(), "0.0 0.0 0.0 0.0"));
                uInsert(customParameters, ParametersPair(Parameters::kMemGenerateIds(), "false"));
                uInsert(customParameters, ParametersPair(Parameters::kLccBowMinInliers(), "4"));
                uInsert(customParameters, ParametersPair(Parameters::kLccBowIterations(), "2000"));
                uInsert(customParameters, ParametersPair(Parameters::kLccBowPnPReprojError(), "1.0"));
                uInsert(customParameters, ParametersPair(Parameters::kLccBowPnPFlags(), "0")); // 0=Iterative, 1=EPNP, 2=P3P
                
                MemoryLoc memoryLoc(customParameters);

                std::string rejectedMsg;
                int visualInliers = 0;
                double variance = 1;
                bool success = true;
                SensorData dataFrom = data;
                dataFrom.setId(newS->id());
                if(dataFrom.id() == Memory::kIdInvalid) {
                    success = false;
                }

                if (success) {
                    std::vector<int> sortedIds = topIds;
                    std::sort(sortedIds.begin(), sortedIds.end());
                    for(std::vector<int>::const_iterator it = sortedIds.begin(); it != sortedIds.end(); ++it) {
                        SensorData data = memory_->getNodeData(*it, true);
                        const Signature * sig = memory_->getSignature(*it);

                        if(!data.depthOrRightRaw().empty() &&
                           data.id() != Memory::kIdInvalid &&
                           sig != NULL)
                        {
                            UDEBUG("Calculate map transform with raw data");
                            //std::cout << "pose before being added: " << dataToS->getPose() << std::endl;
                            memoryLoc.update(data, sig->getPose(), sig->getPoseCovariance());
                        }
                        else
                        {
                            UWARN("Data incomplete. data.depthOrRightRaw().empty() = %d, dataFrom.id() = %d, data.id() = %d", 
                                    data.depthOrRightRaw().empty(), dataFrom.id(), data.id());
                            success = false;
                            break;
                        }
                    }
                    
                    memoryLoc.update(dataFrom);
                }

                if (success) {
                    Transform globalTransform = memoryLoc.computeGlobalVisualTransform(topIds, dataFrom.id(), &rejectedMsg, &visualInliers, &variance);
                    //Transform transform = memoryLoc.computeVisualTransform(topIds[0], dataFrom.id(), &rejectedMsg, &visualInliers, &variance);

                    //float x, y, z, roll, pitch, yaw;
                    //transform.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
                    //transformFile << infoErr->fileName << ", " << topIds[0] << ", " << x << ", " << y << ", " << z << ", " << roll << ", " << pitch << ", " << yaw << ", " << variance << std::endl;

                    if(!globalTransform.isNull()) {
                        //const Signature * mostSimilarS = memory_->getSignature(topIds[0]);
                        //output = mostSimilarS->getPose() * transform.inverse(); // this is the final R and t
                        output = globalTransform;

                        /*
                        CameraModel cmOld = mostSimilarS->sensorData().cameraModels()[0];
                        UDEBUG("cmOld.fx() = %f, cmOld.fy() = %f, cmOld.cx() = %f, cmOld.cy() = %f, cmOld.localTranform() = %s", cmOld.fx(), cmOld.fy(), cmOld.cx(), cmOld.cy(), cmOld.localTransform().prettyPrint().c_str());
                        CameraModel cmNew = newS->sensorData().cameraModels()[0];
                        UDEBUG("cmNew.fx() = %f, cmNew.fy() = %f, cmNew.cx() = %f, cmNew.cy() = %f, cmNew.localTranform() = %s", cmNew.fx(), cmNew.fy(), cmNew.cx(), cmNew.cy(), cmNew.localTransform().prettyPrint().c_str());
                        */
                        //UINFO("mostSimilarS->getPose() = %s", mostSimilarS->getPose().prettyPrint().c_str());
                        //UINFO("transform = %s", transform.prettyPrint().c_str());
                        UDEBUG("global transform = %s", globalTransform.prettyPrint().c_str());
                        //UINFO("transform.inverse() = %s", transform.inverse().prettyPrint().c_str());
                        //UDEBUG("newS->getPose() = %s", newS->getPose().prettyPrint().c_str());
                        //UDEBUG("newS->getPose().inverse() = %s", newS->getPose().inverse().prettyPrint().c_str());
                    }
                    else
                    {
                        UWARN("transform is null, rejectMsg = %s", rejectedMsg.c_str());
                    }
                }
            }
            else
            {
                UWARN("new signature doesn't have enough words. newWords=%d ", (int)newS->getWords().size());
            }

            // remove new words from dictionary
            memory_->deleteLocation(newS->id());
        }
    }
    else
    {
        UERROR("Memory not initialized. memory_->getWorkingMem().size() = %d", memory_->getWorkingMem().size());
    }

    memory_->emptyTrash();

    if(this->isInfoDataFilled() && info)
    {
        // TODO is this function returning global transform?
    }

    UINFO("output transform = %s", output.prettyPrint().c_str());

    return output;
}

// same function copied from RTABMap
void OdometrySporadic::adjustLikelihood(std::map<int, float> & likelihood) const
{
    ULOGGER_DEBUG("likelihood.size()=%d", likelihood.size());
    UTimer timer;
    timer.start();
    if(likelihood.size()==0)
    {
        return;
    }

    // Use only non-null values (ignore virtual place)
    std::list<float> values;
    bool likelihoodNullValuesIgnored = true;
    for(std::map<int, float>::iterator iter = ++likelihood.begin(); iter!=likelihood.end(); ++iter)
    {
        if((iter->second >= 0 && !likelihoodNullValuesIgnored) ||
           (iter->second > 0 && likelihoodNullValuesIgnored))
        {
            values.push_back(iter->second);
        }
    }
    UDEBUG("values.size=%d", values.size());

    float mean = uMean(values);
    float stdDev = std::sqrt(uVariance(values, mean));


    //Adjust likelihood with mean and standard deviation (see Angeli phd)
    float epsilon = 0.0001;
    float max = 0.0f;
    int maxId = 0;
    for(std::map<int, float>::iterator iter=++likelihood.begin(); iter!= likelihood.end(); ++iter)
    {
        float value = iter->second;
        if(value > mean+stdDev && mean)
        {
            iter->second = (value-(stdDev-epsilon))/mean;
            if(value > max)
            {
                max = value;
                maxId = iter->first;
            }
        }
        else if(value == 1.0f && stdDev == 0)
        {
            iter->second = 1.0f;
            if(value > max)
            {
                max = value;
                maxId = iter->first;
            }
        }
        else
        {
            iter->second = 1.0f;
        }
    }

    if(stdDev > epsilon && max)
    {
        likelihood.begin()->second = mean/stdDev + 1.0f;
    }
    else
    {
        likelihood.begin()->second = 2.0f; //2 * std dev
    }

    double time = timer.ticks();
    UDEBUG("mean=%f, stdDev=%f, max=%f, maxId=%d, time=%fs", mean, stdDev, max, maxId, time);
}

bool OdometrySporadic::compareLikelihood(std::pair<const int, float> const& l, std::pair<const int, float> const& r) {
    return l.second > r.second;
}

} // namespace rtabmap
