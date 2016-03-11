#include <rtabmap/core/Memory.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/core/VWDictionary.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Graph.h>

#include "Localization.h"


namespace rtabmap
{

Localization::Localization(const std::string dbPath, const rtabmap::ParametersMap &parameters) :
    _dbPath(dbPath),
    _topk(TOP_K)
{
    // Setup memory
    _memoryParams.insert(ParametersPair(Parameters::kMemRehearsalSimilarity(), "1.0")); // desactivate rehearsal
    _memoryParams.insert(ParametersPair(Parameters::kMemBinDataKept(), "false"));
    _memoryParams.insert(ParametersPair(Parameters::kMemImageKept(), "true"));
    _memoryParams.insert(ParametersPair(Parameters::kMemSTMSize(), "0"));
    _memoryParams.insert(ParametersPair(Parameters::kMemNotLinkedNodesKept(), "false"));
    _memoryParams.insert(ParametersPair(Parameters::kKpTfIdfLikelihoodUsed(), "false"));
    _memoryParams.insert(ParametersPair(Parameters::kKpDetectorStrategy(), uNumber2Str(Feature2D::kFeatureSurf)));
    // parameters that makes memory do PnP localization for RGB images
    _memoryParams.insert(ParametersPair(Parameters::kLccBowEstimationType(), "1")); // 1 is PnP
    _memoryParams.insert(ParametersPair(Parameters::kMemIncrementalMemory(), "false"));
    _memoryParams.insert(ParametersPair(Parameters::kLccBowMinInliers(), "20"));

    _memory = new MemoryLoc();
    if (!_memory || !_memory->init(_dbPath, false, _memoryParams))
    {
        UERROR("Error initializing the memory for Localization.");
    }

    optimize();

    //_memory->generateImages(); // generate synthetic images

    _memoryLocParams.insert(ParametersPair(Parameters::kMemIncrementalMemory(), "true")); // make sure it is incremental
    _memoryLocParams.insert(ParametersPair(Parameters::kMemRehearsalSimilarity(), "1.0")); // desactivate rehearsal
    _memoryLocParams.insert(ParametersPair(Parameters::kMemBinDataKept(), "false"));
    _memoryLocParams.insert(ParametersPair(Parameters::kMemSTMSize(), "0"));
    _memoryLocParams.insert(ParametersPair(Parameters::kKpIncrementalDictionary(), "true")); // make sure it is incremental
    _memoryLocParams.insert(ParametersPair(Parameters::kKpNewWordsComparedTogether(), "false"));
    _memoryLocParams.insert(ParametersPair(Parameters::kKpNNStrategy(), uNumber2Str(VWDictionary::kNNBruteForce))); // bruteforce
    _memoryLocParams.insert(ParametersPair(Parameters::kKpNndrRatio(), "0.3"));
    _memoryLocParams.insert(ParametersPair(Parameters::kKpDetectorStrategy(), uNumber2Str(Feature2D::kFeatureSurf)));
    _memoryLocParams.insert(ParametersPair(Parameters::kKpWordsPerImage(), "1500"));
    _memoryLocParams.insert(ParametersPair(Parameters::kKpBadSignRatio(), "0"));
    _memoryLocParams.insert(ParametersPair(Parameters::kKpRoiRatios(), "0.0 0.0 0.0 0.0"));
    _memoryLocParams.insert(ParametersPair(Parameters::kMemGenerateIds(), "false"));
    _memoryLocParams.insert(ParametersPair(Parameters::kLccBowMinInliers(), "4"));
    _memoryLocParams.insert(ParametersPair(Parameters::kLccBowIterations(), "2000"));
    _memoryLocParams.insert(ParametersPair(Parameters::kLccBowPnPReprojError(), "1.0"));
    _memoryLocParams.insert(ParametersPair(Parameters::kLccBowPnPFlags(), "0")); // 0=Iterative, 1=EPNP, 2=P3P
}

Localization::~Localization()
{
    delete _memory;
}

Transform Localization::localize(const SensorData &data_)
{
    Transform output;
    SensorData data = data_; // copy because it will be changed later

    UASSERT(!data.imageRaw().empty());

    if (!data.stereoCameraModel().isValid() &&
            (data.cameraModels().size() == 0 || !data.cameraModels()[0].isValid()))
    {
        UERROR("Rectified images required! Calibrate your camera.");
        return Transform();
    }

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

    if (_memory->getWorkingMem().size() >= 1)
    {
        // generate kpts
        if (_memory->update(data))
        {
            UDEBUG("");
            const Signature *newS = _memory->getLastWorkingSignature();
            UDEBUG("newWords=%d", (int)newS->getWords().size());
            int bowMinInliers;
            Parameters::parse(_memoryLocParams, Parameters::kLccBowMinInliers(), bowMinInliers);
            if ((int)newS->getWords().size() > bowMinInliers)
            {
                std::map<int, float> likelihood;
                std::list<int> signaturesToCompare = uKeysList(_memory->getWorkingMem());
                UDEBUG("signaturesToCompare.size() = %d", signaturesToCompare.size());
                likelihood = _memory->computeLikelihood(newS, signaturesToCompare);

                Rtabmap rtabmap;
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
                        SensorData data = _memory->getNodeData(*it, true);
                        const Signature *sig = _memory->getSignature(*it);

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

void Localization::optimize()
{
    // get the graph
    std::map<int, int> ids = _memory->getNeighborsId(_memory->getLastSignatureId(), 0, 0);
    std::map<int, Transform> poses;
    std::multimap<int, Link> links;
    _memory->getMetricConstraints(uKeysSet(ids), poses, links);

    //optimize the graph
    graph::TOROOptimizer optimizer;
    _optimizedPoses = optimizer.optimize(poses.begin()->first, poses, links);
}

bool Localization::compareLikelihood(std::pair<const int, float> const &l, std::pair<const int, float> const &r)
{
    return l.second > r.second;
}

} // namespace rtabmap
