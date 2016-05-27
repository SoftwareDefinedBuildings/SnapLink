#pragma once

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/Features2d.h>
#include <rtabmap/utilite/UStl.h>
#include <typeinfo>
#include <list>
#include <map>
#include <set>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "VWDictFixed.h"

class VWDictFixed;

class MemoryLoc
{
public:
    static const int kIdStart;

public:
    MemoryLoc();
    virtual ~MemoryLoc();

    bool init(std::vector<std::string> &dbUrls,
              const rtabmap::ParametersMap &parameters = rtabmap::ParametersMap());
    const rtabmap::Signature *createSignature(rtabmap::SensorData &data, void *context);
    void close();

    rtabmap::Signature *getSignature(int dbId, int sigId) const;
    const std::map<int, rtabmap::Signature *> &getSignatureMap(int dbId) const;
    const std::vector<std::map<int, rtabmap::Signature *> > &getSignatureMaps() const;

private:
    virtual void parseParameters(const rtabmap::ParametersMap &parameters);
    std::map<int, rtabmap::Transform> optimizeGraph(int dbId, int rootSigId);  // optimize poses using TORO graph

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

    void moveToTrash(int dbId, rtabmap::Signature *s, bool keepLinkedToGraph = true);
    void removeVirtualLinks(int signatureId);

    int getNextId();
    void clear();


    //keypoint stuff
    void disableWordsRef(int dbId, int signatureId);
    void cleanUnusedWords();

private:
    // parameters
    rtabmap::ParametersMap parameters_;

    std::vector<std::map<int, rtabmap::Signature *> > _signatureMaps;

    //Keypoint stuff
    VWDictFixed *_vwd;
    rtabmap::Feature2D *_feature2D;
};
