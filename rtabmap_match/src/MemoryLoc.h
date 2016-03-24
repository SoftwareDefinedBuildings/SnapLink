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
class Registration;
class RegistrationInfo;
class RegistrationIcp;

class MemoryLoc
{
public:
    static const int kIdStart;
    static const int kIdVirtual;
    static const int kIdInvalid;

public:
    MemoryLoc();
    virtual ~MemoryLoc();

    bool update(const rtabmap::SensorData &data,
                const rtabmap::Transform &pose = rtabmap::Transform(),
                const cv::Mat &covariance = cv::Mat());
    bool init(const std::string &dbUrl,
              const rtabmap::ParametersMap &parameters = rtabmap::ParametersMap());
    void close();
    std::map<int, float> computeLikelihood(const rtabmap::Signature *signature,
                                           const std::list<int> &ids);

    int cleanup();
    void emptyTrash();
    void removeVirtualLinks(int signatureId);
    std::map<int, int> getNeighborsId(
        int signatureId,
        int maxGraphDepth,
        int maxCheckedInDatabase = -1,
        bool incrementMarginOnLoop = false,
        bool ignoreLoopIds = false,
        bool ignoreIntermediateNodes = false,
        double *dbAccessTime = 0) const;
    void deleteLocation(int locationId, std::list<int> *deletedWords = 0);
    void removeLink(int idA, int idB);

    //getters
    const std::map<int, double> &getWorkingMem() const
    {
        return _workingMem;
    }
    std::map<int, rtabmap::Link> getNeighborLinks(int signatureId,
            bool lookInDatabase = false) const;
    std::map<int, rtabmap::Link> getLinks(int signatureId,
                                          bool lookInDatabase = false) const;
    const rtabmap::Signature *getLastWorkingSignature() const;
    rtabmap::Transform getOdomPose(int signatureId, bool lookInDatabase = false) const;
    rtabmap::Transform getGroundTruthPose(int signatureId, bool lookInDatabase = false) const;
    bool getNodeInfo(int signatureId,
                     rtabmap::Transform &odomPose,
                     int &mapId,
                     int &weight,
                     std::string &label,
                     double &stamp,
                     rtabmap::Transform &groundTruth,
                     bool lookInDatabase = false) const;
    rtabmap::SensorData getNodeData(int nodeId, bool uncompressedData = false, bool keepLoadedDataInMemory = true);
    const rtabmap::Signature *getSignature(int id) const;
    const std::map<int, rtabmap::Signature *> &getSignatures() const
    {
        return _signatures;
    }

    // RGB-D stuff
    void getMetricConstraints(
        const std::set<int> &ids,
        std::map<int, rtabmap::Transform> &poses,
        std::multimap<int, rtabmap::Link> &links,
        bool lookInDatabase = false);

    rtabmap::Transform computeGlobalVisualTransform(const std::vector<int> &oldIds, int newId) const;
    rtabmap::Transform computeGlobalVisualTransform(const std::vector<const rtabmap::Signature *> &oldSigs, const rtabmap::Signature *newSig) const;

private:
    virtual void parseParameters(const rtabmap::ParametersMap &parameters);
    void preUpdate();
    void addSignature(rtabmap::Signature *signature);
    void moveToTrash(rtabmap::Signature *s, bool keepLinkedToGraph = true, std::list<int> *deletedWords = 0);

    rtabmap::Signature *_getSignature(int id) const;
    int getNextId();
    int incrementMapId();
    void clear();
    void initCountId();

    void copyData(const rtabmap::Signature *from, rtabmap::Signature *to);
    rtabmap::Signature *createSignature(
        const rtabmap::SensorData &data,
        const rtabmap::Transform &pose);

    //keypoint stuff
    void disableWordsRef(int signatureId);
    void enableWordsRef(const std::list<int> &signatureIds);
    void cleanUnusedWords();
    int getNi(int signatureId) const;

protected:
    rtabmap::DBDriver *_dbDriver;

private:
    // parameters
    rtabmap::ParametersMap parameters_;
    bool _rawDescriptorsKept;
    bool _notLinkedNodesKeptInDb;
    bool _incrementalMemory;
    int _maxStMemSize;
    float _recentWmRatio;
    bool _transferSortingByWeightId;
    bool _generateIds;
    bool _badSignaturesIgnored;
    bool _mapLabelsAdded;
    int _imageDecimation;
    float _laserScanDownsampleStepSize;
    bool _useOdometryFeatures;

    int _idCount;
    int _idMapCount;
    rtabmap::Signature *_lastSignature;
    int _lastGlobalLoopClosureId;
    bool _memoryChanged; // False by default, become true only when MemoryLoc::update() is called.
    bool _linksChanged; // False by default, become true when links are modified.

    std::map<int, rtabmap::Signature *> _signatures; // TODO : check if a signature is already added? although it is not supposed to occur...
    std::map<int, double> _workingMem; // id,age

    //Keypoint stuff
    rtabmap::VWDictionary *_vwd;
    rtabmap::Feature2D *_feature2D;
    bool _tfIdfLikelihoodUsed;

    int _minInliers;
    int _iterations;
    int _pnpRefineIterations;
    double _pnpReprojError;
    int _pnpFlags;
};
