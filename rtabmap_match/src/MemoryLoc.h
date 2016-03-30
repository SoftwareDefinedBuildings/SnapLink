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

    bool update(const rtabmap::SensorData &data);
    bool init(const std::string &dbUrl,
              const rtabmap::ParametersMap &parameters = rtabmap::ParametersMap());
    void close();
    std::map<int, float> computeLikelihood(const rtabmap::Signature *signature,
                                           const std::list<int> &ids);

    void emptyTrash();
    void deleteLocation(int locationId);

    const rtabmap::Signature *getLastWorkingSignature() const;
    rtabmap::Transform getOptimizedPose(int signatureId) const;
    const rtabmap::Signature *getSignature(int id) const;
    const std::map<int, rtabmap::Signature *> &getSignatures() const
    {
        return _signatures;
    }

    rtabmap::Transform computeGlobalVisualTransform(const std::vector<int> &oldIds, int newId) const;

private:
    virtual void parseParameters(const rtabmap::ParametersMap &parameters);
    void optimizeGraph();  // optimize poses using TORO graph

    std::map<int, int> getNeighborsId(
        int signatureId,
        int maxGraphDepth,
        int maxCheckedInDatabase = -1,
        bool incrementMarginOnLoop = false,
        bool ignoreLoopIds = false,
        bool ignoreIntermediateNodes = false,
        double *dbAccessTime = 0) const;
    std::map<int, rtabmap::Link> getLinks(int signatureId,
                                          bool lookInDatabase = false) const;
    std::map<int, rtabmap::Link> getNeighborLinks(int signatureId) const;
    rtabmap::Transform getOdomPose(int signatureId, bool lookInDatabase = false) const;
    bool getNodeInfo(int signatureId,
                     rtabmap::Transform &odomPose,
                     int &mapId,
                     int &weight,
                     std::string &label,
                     double &stamp,
                     rtabmap::Transform &groundTruth,
                     bool lookInDatabase = false) const;
    void getMetricConstraints(
        const std::set<int> &ids,
        std::map<int, rtabmap::Transform> &poses,
        std::multimap<int, rtabmap::Link> &links,
        bool lookInDatabase = false);


    void addSignature(rtabmap::Signature *signature);
    void moveToTrash(rtabmap::Signature *s, bool keepLinkedToGraph = true);
    void removeVirtualLinks(int signatureId);

    rtabmap::Signature *_getSignature(int id) const;
    int getNextId();
    void clear();

    rtabmap::Signature *createSignature(const rtabmap::SensorData &data);

    //keypoint stuff
    void disableWordsRef(int signatureId);
    void cleanUnusedWords();

    rtabmap::Transform computeGlobalVisualTransform(const std::vector<const rtabmap::Signature *> &oldSigs, const rtabmap::Signature *newSig) const;

protected:
    rtabmap::DBDriver *_dbDriver;

private:
    // parameters
    rtabmap::ParametersMap parameters_;
    bool _badSignaturesIgnored;

    int _idCount;

    std::map<int, rtabmap::Transform> _optimizedPoses;
    std::map<int, rtabmap::Signature *> _signatures; // TODO : check if a signature is already added? although it is not supposed to occur...

    //Keypoint stuff
    rtabmap::VWDictionary *_vwd;
    rtabmap::Feature2D *_feature2D;

    int _minInliers;
    int _iterations;
    int _pnpRefineIterations;
    double _pnpReprojError;
    int _pnpFlags;
};
