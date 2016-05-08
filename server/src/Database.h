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

// the database is read-only for now
class Database
{
public:
    static const int kIdStart;

public:
    Database();
    virtual ~Database();

    bool init(const std::string &dbUrl,
              const rtabmap::ParametersMap &parameters = rtabmap::ParametersMap());
    void close();

    const rtabmap::Signature *getLastWorkingSignature() const;
    rtabmap::Transform getOptimizedPose(int signatureId) const;
    const rtabmap::Signature *getSignature(int id) const;
    const std::map<int, rtabmap::Signature *> &getSignatures() const;

private:
    virtual void parseParameters(const rtabmap::ParametersMap &parameters);
    void optimizeGraph();  // optimize poses using TORO graph

    std::map<int, int> getNeighborsId(
        int signatureId,
        int maxGraphDepth,
        bool incrementMarginOnLoop = false,
        bool ignoreLoopIds = false,
        bool ignoreIntermediateNodes = false) const;
    std::map<int, rtabmap::Link> getNeighborLinks(int signatureId) const;
    void getMetricConstraints(
        const std::set<int> &ids,
        std::map<int, rtabmap::Transform> &poses,
        std::multimap<int, rtabmap::Link> &links);


    void moveToTrash(rtabmap::Signature *s, bool keepLinkedToGraph = true);

    rtabmap::Signature *_getSignature(int id) const;
    void clear();

    //keypoint stuff
    void disableWordsRef(int signatureId);
    void cleanUnusedWords();

protected:
    rtabmap::DBDriver *_dbDriver;

private:
    // parameters
    rtabmap::ParametersMap parameters_;
    bool _badSignaturesIgnored;

    std::map<int, rtabmap::Transform> _optimizedPoses;
    std::map<int, rtabmap::Signature *> _signatures; // TODO : check if a signature is already added? although it is not supposed to occur...

    //Keypoint stuff
    rtabmap::VWDictionary *_vwd;
    rtabmap::Feature2D *_feature2D;
};
