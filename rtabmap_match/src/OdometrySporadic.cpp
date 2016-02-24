#include <rtabmap/core/Memory.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/core/VWDictionary.h>
#include <rtabmap/core/Rtabmap.h>

#include "OdometrySporadic.h"


namespace rtabmap
{

OdometrySporadic::OdometrySporadic(const std::string dbPath, const rtabmap::ParametersMap &parameters) :
    Odometry(parameters),
    dbPath_(dbPath)
{
    // Setup memory
    memoryParams_.insert(ParametersPair(Parameters::kMemRehearsalSimilarity(), "1.0")); // desactivate rehearsal
    memoryParams_.insert(ParametersPair(Parameters::kMemBinDataKept(), "false"));
    memoryParams_.insert(ParametersPair(Parameters::kMemImageKept(), "true"));
    memoryParams_.insert(ParametersPair(Parameters::kMemSTMSize(), "0"));
    memoryParams_.insert(ParametersPair(Parameters::kMemNotLinkedNodesKept(), "false"));
    memoryParams_.insert(ParametersPair(Parameters::kKpTfIdfLikelihoodUsed(), "false"));
    memoryParams_.insert(ParametersPair(Parameters::kKpDetectorStrategy(), uNumber2Str(Feature2D::kFeatureSurf)));
    // parameters that makes memory do PnP localization for RGB images
    memoryParams_.insert(ParametersPair(Parameters::kLccBowEstimationType(), "1")); // 1 is PnP
    memoryParams_.insert(ParametersPair(Parameters::kMemIncrementalMemory(), "false"));
    memoryParams_.insert(ParametersPair(Parameters::kLccBowMinInliers(), "20"));

    memory_ = new MemoryLoc();
    if (!memory_ || !memory_->init(dbPath_, false, memoryParams_))
    {
        UERROR("Error initializing the memory for OdometrySporadic.");
    }
    //memory_->generateImages(); // generate synthetic images

    memoryLocParams_.insert(ParametersPair(Parameters::kMemIncrementalMemory(), "true")); // make sure it is incremental
    memoryLocParams_.insert(ParametersPair(Parameters::kMemRehearsalSimilarity(), "1.0")); // desactivate rehearsal
    memoryLocParams_.insert(ParametersPair(Parameters::kMemBinDataKept(), "false"));
    memoryLocParams_.insert(ParametersPair(Parameters::kMemSTMSize(), "0"));
    memoryLocParams_.insert(ParametersPair(Parameters::kKpIncrementalDictionary(), "true")); // make sure it is incremental
    memoryLocParams_.insert(ParametersPair(Parameters::kKpNewWordsComparedTogether(), "false"));
    memoryLocParams_.insert(ParametersPair(Parameters::kKpNNStrategy(), uNumber2Str(VWDictionary::kNNBruteForce))); // bruteforce
    memoryLocParams_.insert(ParametersPair(Parameters::kKpNndrRatio(), "0.3"));
    memoryLocParams_.insert(ParametersPair(Parameters::kKpDetectorStrategy(), uNumber2Str(Feature2D::kFeatureSurf)));
    memoryLocParams_.insert(ParametersPair(Parameters::kKpWordsPerImage(), "1500"));
    memoryLocParams_.insert(ParametersPair(Parameters::kKpBadSignRatio(), "0"));
    memoryLocParams_.insert(ParametersPair(Parameters::kKpRoiRatios(), "0.0 0.0 0.0 0.0"));
    memoryLocParams_.insert(ParametersPair(Parameters::kMemGenerateIds(), "false"));
    memoryLocParams_.insert(ParametersPair(Parameters::kLccBowMinInliers(), "4"));
    memoryLocParams_.insert(ParametersPair(Parameters::kLccBowIterations(), "2000"));
    memoryLocParams_.insert(ParametersPair(Parameters::kLccBowPnPReprojError(), "1.0"));
    memoryLocParams_.insert(ParametersPair(Parameters::kLccBowPnPFlags(), "0")); // 0=Iterative, 1=EPNP, 2=P3P
}

OdometrySporadic::~OdometrySporadic()
{
    delete memory_;
}

void OdometrySporadic::reset(const Transform &initialPose)
{
    Odometry::reset(initialPose);
}

Transform OdometrySporadic::computeTransform(const SensorData &data_, OdometryInfo *info)
{
    Transform output;
    SensorData data = data_; // copy because it will be changed later

    if (data.imageRaw().empty())
    {
        UERROR("Image empty! Cannot compute odometry...");
        return output;
    }
    if (!((data.cameraModels().size() == 1 && data.cameraModels()[0].isValid()) || data.stereoCameraModel().isValid()))
    {
        UERROR("Odometry cannot be done without calibration or on multi-camera!");
        return output;
    }

    if (data.imageRaw().channels() > 1)
    {
        cv::Mat imageRawGray;
        cv::cvtColor(data.imageRaw(), imageRawGray, cv::COLOR_BGR2GRAY);
        data.setImageRaw(imageRawGray);
    }

    const CameraModel &cameraModel = data.stereoCameraModel().isValid() ? data.stereoCameraModel().left() : data.cameraModels()[0];

    UTimer timer;

    if (memory_->getWorkingMem().size() >= 1)
    {
        //PnP
        if (this->isInfoDataFilled() && info)
        {
            info->type = -1; // 0=BOW, 1=Optical Flow, 2=ICP
        }

        // generate kpts
        if (memory_->update(data))
        {
            UDEBUG("");
            const Signature *newS = memory_->getLastWorkingSignature();
            UDEBUG("newWords=%d", (int)newS->getWords().size());
            if ((int)newS->getWords().size() > this->getMinInliers())
            {
                std::map<int, float> likelihood;
                std::list<int> signaturesToCompare = uKeysList(memory_->getWorkingMem());
                UDEBUG("signaturesToCompare.size() = %d", signaturesToCompare.size());
                likelihood = memory_->computeLikelihood(newS, signaturesToCompare);

                Rtabmap rtabmap;
                rtabmap.adjustLikelihood(likelihood);

                std::vector<int> topIds;
                likelihood.erase(-1);
                int topk_ = 1;
                int topId;
                if (likelihood.size())
                {
                    std::vector< std::pair<int, float> > top(topk_);
                    std::partial_sort_copy(likelihood.begin(),
                                           likelihood.end(),
                                           top.begin(),
                                           top.end(),
                                           compareLikelihood);
                    // TODO there is some bugs here
                    for (std::vector< std::pair<int, float> >::iterator it = top.begin(); it != top.end(); ++it)
                    {
                        topIds.push_back(it->first);
                    }
                    topId = topIds[0];
                    UINFO("topId: %d", topId);
                }

                MemoryLoc memoryLoc(memoryLocParams_);

                std::string rejectedMsg;
                int visualInliers = 0;
                double variance = 1;
                bool success = true;
                data.setId(newS->id());
                if (data.id() == Memory::kIdInvalid)
                {
                    success = false;
                }

                if (success)
                {
                    std::vector<int> sortedIds = topIds;
                    std::sort(sortedIds.begin(), sortedIds.end());
                    for (std::vector<int>::const_iterator it = sortedIds.begin(); it != sortedIds.end(); ++it)
                    {
                        SensorData data = memory_->getNodeData(*it, true);
                        const Signature *sig = memory_->getSignature(*it);

                        if (!data.depthOrRightRaw().empty() &&
                                data.id() != Memory::kIdInvalid &&
                                sig != NULL)
                        {
                            UDEBUG("Calculate map transform with raw data");
                            //std::cout << "pose before being added: " << dataToS->getPose() << std::endl;
                            memoryLoc.update(data, sig->getPose(), sig->getPoseCovariance());
                        }
                        else
                        {
                            UWARN("Data incomplete. data.depthOrRightRaw().empty() = %d, data.id() = %d",
                                  data.depthOrRightRaw().empty(), data.id());
                            success = false;
                            break;
                        }
                    }

                    memoryLoc.update(data);
                }

                if (success)
                {
                    output = memoryLoc.computeGlobalVisualTransform(topIds, data.id(), &rejectedMsg, &visualInliers, &variance);

                    if (!output.isNull())
                    {
                        UDEBUG("global transform = %s", output.prettyPrint().c_str());
                    }
                    else
                    {
                        UWARN("transform is null, rejectMsg = %s, using pose of the closest image", rejectedMsg.c_str());
                        output = memory_->getSignature(topId)->getPose();
                    }
                }
            }
            else
            {
                UWARN("new signature doesn't have enough words. newWords=%d ", (int)newS->getWords().size());
            }

            // remove new words from dictionary
            memory_->deleteLocation(newS->id());
            memory_->emptyTrash();
        }
    }
    else
    {
        UERROR("Memory not initialized. memory_->getWorkingMem().size() = %d", memory_->getWorkingMem().size());
    }

    if (this->isInfoDataFilled() && info)
    {
        // TODO
    }

    UINFO("output transform = %s", output.prettyPrint().c_str());

    return output;
}

bool OdometrySporadic::compareLikelihood(std::pair<const int, float> const &l, std::pair<const int, float> const &r)
{
    return l.second > r.second;
}

} // namespace rtabmap
