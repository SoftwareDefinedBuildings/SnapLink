#pragma once

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/Features2d.h>
#include <rtabmap/core/VWDictionary.h>
#include <rtabmap/utilite/UStl.h>
#include <typeinfo>
#include <list>
#include <map>
#include <set>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

class Signature;
class DBDriver;
class VWDictionary;
class VisualWord;
class Feature2D;

class MemoryLoc
{
public:
    static const int kIdStart;

public:
    MemoryLoc();
    virtual ~MemoryLoc();

    bool init(std::vector<std::string> &dbUrls,
              const rtabmap::ParametersMap &parameters = rtabmap::ParametersMap());
    int add(rtabmap::SensorData &data, void *context);
    void remove(int locationId);
    void close();

    rtabmap::Transform getOptimizedPose(int signatureId) const;
    const rtabmap::Signature *getSignature(int dbId, int sigId) const;
    const std::map<int, rtabmap::Signature *> &getSignatureMap(int dbId) const;

private:
    virtual void parseParameters(const rtabmap::ParametersMap &parameters);
    void optimizeGraph(int dbId);  // optimize poses using TORO graph

    std::map<int, int> getNeighborsId(
        int dbId,
        int signatureId,
        bool incrementMarginOnLoop = false,
        bool ignoreLoopIds = false,
        bool ignoreIntermediateNodes = false) const;
    std::map<int, rtabmap::Link> getNeighborLinks(int dbId, int sigId) const;
    void getMetricConstraints(
        int dbId,
        const std::set<int> &ids,
        std::map<int, rtabmap::Transform> &poses,
        std::multimap<int, rtabmap::Link> &links);


    void moveToTrash(rtabmap::Signature *s, bool keepLinkedToGraph = true);
    void removeVirtualLinks(int signatureId);

    int getNextId();
    void clear();

    rtabmap::Signature *createSignature(rtabmap::SensorData &data, void *context);

    //keypoint stuff
    void disableWordsRef(int signatureId);
    void cleanUnusedWords();

private:
    // parameters
    rtabmap::ParametersMap parameters_;
    bool _badSignaturesIgnored;

    std::vector<int> _idCounts;

    std::vector<std::map<int, rtabmap::Signature *> > _signatureMaps;
    std::vector<std::map<int, rtabmap::Transform> > _optimizedPoseMaps;

    //Keypoint stuff
    rtabmap::VWDictionary *_vwd;
    rtabmap::Feature2D *_feature2D;
};
