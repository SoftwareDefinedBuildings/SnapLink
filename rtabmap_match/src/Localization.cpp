#include <rtabmap/core/Memory.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/core/VWDictionary.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Optimizer.h>

#include "Localization.h"

Localization::Localization(const std::string dbPath, const rtabmap::ParametersMap &parameters) :
    _dbPath(dbPath),
    _topk(TOP_K)
{
    // Setup memory
    _memoryParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemRehearsalSimilarity(), "1.0")); // desactivate rehearsal
    _memoryParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemBinDataKept(), "false"));
    _memoryParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemImageKept(), "true"));
    _memoryParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemSTMSize(), "0"));
    _memoryParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemNotLinkedNodesKept(), "false"));
    _memoryParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpTfIdfLikelihoodUsed(), "false"));
    _memoryParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpDetectorStrategy(), uNumber2Str(rtabmap::Feature2D::kFeatureSurf)));
    // parameters that makes memory do PnP localization for RGB images
    _memoryParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisEstimationType(), "1")); // Motion estimation approach: 0:3D->3D, 1:3D->2D (PnP), 2:2D->2D (Epipolar Geometry)
    _memoryParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemIncrementalMemory(), "false"));
    _memoryParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisMinInliers(), "20"));

    _memory = new MemoryLoc();
    if (!_memory || !_memory->init(_dbPath, false, _memoryParams))
    {
        UERROR("Error initializing the memory for Localization.");
    }

    optimizeGraph();

    _memoryLocParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemIncrementalMemory(), "true")); // make sure it is incremental
    _memoryLocParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemRehearsalSimilarity(), "1.0")); // desactivate rehearsal
    _memoryLocParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemBinDataKept(), "false"));
    _memoryLocParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemSTMSize(), "0"));
    _memoryLocParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpIncrementalDictionary(), "true")); // make sure it is incremental
    _memoryLocParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpNewWordsComparedTogether(), "false"));
    _memoryLocParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpNNStrategy(), uNumber2Str(rtabmap::VWDictionary::kNNBruteForce))); // bruteforce
    _memoryLocParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpNndrRatio(), "0.3"));
    _memoryLocParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpDetectorStrategy(), uNumber2Str(rtabmap::Feature2D::kFeatureSurf)));
    _memoryLocParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpMaxFeatures(), "1500"));
    _memoryLocParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpBadSignRatio(), "0"));
    _memoryLocParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpRoiRatios(), "0.0 0.0 0.0 0.0"));
    _memoryLocParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemGenerateIds(), "false"));
    _memoryLocParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisMinInliers(), "4"));
    _memoryLocParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisIterations(), "2000"));
    _memoryLocParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisPnPReprojError(), "1.0"));
    _memoryLocParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisPnPFlags(), "0")); // 0=Iterative, 1=EPNP, 2=P3P
}

Localization::~Localization()
{
    delete _memory;
}

rtabmap::Transform Localization::localize(rtabmap::SensorData data)
{
    rtabmap::Transform output;

    UASSERT(!data.imageRaw().empty());

    if (!data.stereoCameraModel().isValidForProjection() && (data.cameraModels().size() == 0 || !data.cameraModels()[0].isValidForProjection()))
    {
        UERROR("Rectified images required! Calibrate your camera.");
        return output;
    }
    if (data.imageRaw().empty())
    {
        UERROR("Image empty! Cannot compute odometry...");
        return output;
    }
    if (!((data.cameraModels().size() == 1 && data.cameraModels()[0].isValidForProjection()) || data.stereoCameraModel().isValidForProjection()))
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

    const rtabmap::CameraModel &cameraModel = data.stereoCameraModel().isValidForProjection() ? data.stereoCameraModel().left() : data.cameraModels()[0];

    if (_memory->getWorkingMem().size() >= 1)
    {
        // generate kpts
        if (_memory->update(data))
        {
            UDEBUG("");
            const rtabmap::Signature *newS = _memory->getLastWorkingSignature();
            UDEBUG("newWords=%d", (int)newS->getWords().size());
            int minInliers;
            rtabmap::Parameters::parse(_memoryLocParams, rtabmap::Parameters::kVisMinInliers(), minInliers);
            if ((int)newS->getWords().size() > minInliers)
            {
                std::map<int, float> likelihood;
                std::list<int> signaturesToCompare = uKeysList(_memory->getWorkingMem());
                UDEBUG("signaturesToCompare.size() = %d", signaturesToCompare.size());
                likelihood = _memory->computeLikelihood(newS, signaturesToCompare);

                rtabmap::Rtabmap rtabmap;
                rtabmap.adjustLikelihood(likelihood);

                std::vector<int> topIds;
                likelihood.erase(-1);
                int topId;
                if (likelihood.size())
                {
                    std::vector< std::pair<int, float> > top(_topk);
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

                MemoryLoc memoryLoc(_memoryLocParams);

                std::string rejectedMsg;
                int visualInliers = 0;
                double variance = 1;
                bool success = true;
                data.setId(newS->id());
                if (data.id() == rtabmap::Memory::kIdInvalid)
                {
                    success = false;
                }

                if (success)
                {
                    std::vector<int> sortedIds = topIds;
                    std::sort(sortedIds.begin(), sortedIds.end());
                    for (std::vector<int>::const_iterator it = sortedIds.begin(); it != sortedIds.end(); ++it)
                    {
                        rtabmap::SensorData data = _memory->getNodeData(*it, true);
                        const rtabmap::Signature *sig = _memory->getSignature(*it);

                        if (!data.depthOrRightRaw().empty() && data.id() != rtabmap::Memory::kIdInvalid && sig != NULL)
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
                    output = memoryLoc.computeGlobalVisualTransform(topIds, data.id(), &_optimizedPoses, &rejectedMsg, &visualInliers, &variance);

                    if (!output.isNull())
                    {
                        UDEBUG("global transform = %s", output.prettyPrint().c_str());
                    }
                    else
                    {
                        UWARN("transform is null, rejectMsg = %s, using pose of the closest image", rejectedMsg.c_str());
                        output = _memory->getSignature(topId)->getPose();
                    }
                }
            }
            else
            {
                UWARN("new signature doesn't have enough words. newWords=%d ", (int)newS->getWords().size());
            }

            // remove new words from dictionary
            _memory->deleteLocation(newS->id());
            _memory->emptyTrash();
        }
    }
    else
    {
        UERROR("Memory not initialized. _memory->getWorkingMem().size() = %d", _memory->getWorkingMem().size());
    }

    UINFO("output transform = %s", output.prettyPrint().c_str());

    return output;
}

void Localization::optimizeGraph()
{
    // get the graph
    std::list<int> idList = uKeysList(_memory->getWorkingMem());
    std::set<int> idSet(idList.begin(), idList.end());
    std::map<int, rtabmap::Transform> poses;
    std::multimap<int, rtabmap::Link> links;
    bool lookInDatabase = true;
    _memory->getMetricConstraints(idSet, poses, links, lookInDatabase);

    //optimize the graph
    rtabmap::Optimizer::Type optimizerType = rtabmap::Optimizer::kTypeTORO; // options: kTypeTORO, kTypeG2O, kTypeGTSAM, kTypeCVSBA
    rtabmap::Optimizer *graphOptimizer = rtabmap::Optimizer::create(optimizerType);
    _optimizedPoses = graphOptimizer->optimize(poses.begin()->first, poses, links);
}

bool Localization::compareLikelihood(std::pair<const int, float> const &l, std::pair<const int, float> const &r)
{
    return l.second > r.second;
}
