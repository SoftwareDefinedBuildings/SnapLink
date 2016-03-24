#pragma once

#include <rtabmap/utilite/UEventsHandler.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/Features2d.h>
#include <rtabmap/core/Statistics.h>
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
class Statistics;
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

    virtual const rtabmap::ParametersMap &getParameters() const
    {
        return parameters_;
    }
    bool update(const rtabmap::SensorData &data,
                rtabmap::Statistics *stats = 0);
    bool update(const rtabmap::SensorData &data,
                const rtabmap::Transform &pose,
                const cv::Mat &covariance,
                rtabmap::Statistics *stats = 0);
    bool init(const std::string &dbUrl,
              const rtabmap::ParametersMap &parameters = rtabmap::ParametersMap());
    void close();
    std::map<int, float> computeLikelihood(const rtabmap::Signature *signature,
                                           const std::list<int> &ids);
    int incrementMapId(std::map<int, int> *reducedIds = 0);

    int cleanup();
    void emptyTrash();
    void joinTrashThread();
    bool addLink(const rtabmap::Link &link);
    void removeVirtualLinks(int signatureId);
    std::map<int, int> getNeighborsId(
        int signatureId,
        int maxGraphDepth,
        int maxCheckedInDatabase = -1,
        bool incrementMarginOnLoop = false,
        bool ignoreLoopIds = false,
        bool ignoreIntermediateNodes = false,
        double *dbAccessTime = 0) const;
    std::map<int, float> getNeighborsIdRadius(
        int signatureId,
        float radius,
        const std::map<int, rtabmap::Transform> &optimizedPoses,
        int maxGraphDepth) const;
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
    bool isBinDataKept() const
    {
        return _binDataKept;
    }
    float getSimilarityThreshold() const
    {
        return _similarityThreshold;
    }
    const rtabmap::Signature *getLastWorkingSignature() const;
    int getSignatureIdByLabel(const std::string &label, bool lookInDatabase = true) const;
    std::map<int, std::string> getAllLabels() const;
    std::string getDatabaseVersion() const;
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
    void getNodeWords(int nodeId,
                      std::multimap<int, cv::KeyPoint> &words,
                      std::multimap<int, cv::Point3f> &words3);
    std::set<int> getAllSignatureIds() const;
    bool memoryChanged() const
    {
        return _memoryChanged;
    }
    bool isIncremental() const
    {
        return _incrementalMemory;
    }
    const rtabmap::Signature *getSignature(int id) const;
    bool isInSTM(int signatureId) const
    {
        return _stMem.find(signatureId) != _stMem.end();
    }
    bool isInWM(int signatureId) const
    {
        return _workingMem.find(signatureId) != _workingMem.end();
    }
    bool isInLTM(int signatureId) const
    {
        return !this->isInSTM(signatureId) && !this->isInWM(signatureId);
    }
    bool isIDsGenerated() const
    {
        return _generateIds;
    }
    int getLastGlobalLoopClosureId() const
    {
        return _lastGlobalLoopClosureId;
    }
    const rtabmap::Feature2D *getFeature2D() const
    {
        return _feature2D;
    }
    bool isGraphReduced() const
    {
        return _reduceGraph;
    }

    //keypoint stuff
    const rtabmap::VWDictionary *getVWDictionary() const;

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
    void addSignatureToStm(rtabmap::Signature *signature, const cv::Mat &covariance);
    void clear();
    void moveToTrash(rtabmap::Signature *s, bool keepLinkedToGraph = true, std::list<int> *deletedWords = 0);

    void moveSignatureToWMFromSTM(int id, int *reducedTo = 0);
    void addSignatureToWmFromLTM(rtabmap::Signature *signature);
    rtabmap::Signature *_getSignature(int id) const;
    std::list<rtabmap::Signature *> getRemovableSignatures(int count,
            const std::set<int> &ignoredIds = std::set<int>());
    int getNextId();
    void initCountId();
    void rehearsal(rtabmap::Signature *signature, rtabmap::Statistics *stats = 0);
    bool rehearsalMerge(int oldId, int newId);

    const std::map<int, rtabmap::Signature *> &getSignatures() const
    {
        return _signatures;
    }

    void copyData(const rtabmap::Signature *from, rtabmap::Signature *to);
    rtabmap::Signature *createSignature(
        const rtabmap::SensorData &data,
        const rtabmap::Transform &pose,
        rtabmap::Statistics *stats = 0);

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
    float _similarityThreshold;
    bool _binDataKept;
    bool _rawDescriptorsKept;
    bool _saveDepth16Format;
    bool _notLinkedNodesKeptInDb;
    bool _incrementalMemory;
    bool _reduceGraph;
    int _maxStMemSize;
    float _recentWmRatio;
    bool _transferSortingByWeightId;
    bool _idUpdatedToNewOneRehearsal;
    bool _generateIds;
    bool _badSignaturesIgnored;
    bool _mapLabelsAdded;
    int _imageDecimation;
    float _laserScanDownsampleStepSize;
    bool _reextractLoopClosureFeatures;
    float _rehearsalMaxDistance;
    float _rehearsalMaxAngle;
    bool _rehearsalWeightIgnoredWhileMoving;
    bool _useOdometryFeatures;

    int _idCount;
    int _idMapCount;
    rtabmap::Signature *_lastSignature;
    int _lastGlobalLoopClosureId;
    bool _memoryChanged; // False by default, become true only when MemoryLoc::update() is called.
    bool _linksChanged; // False by default, become true when links are modified.
    int _signaturesAdded;

    std::map<int, rtabmap::Signature *> _signatures; // TODO : check if a signature is already added? although it is not supposed to occur...
    std::set<int> _stMem; // id
    std::map<int, double> _workingMem; // id,age

    //Keypoint stuff
    rtabmap::VWDictionary *_vwd;
    rtabmap::Feature2D *_feature2D;
    float _badSignRatio;;
    bool _tfIdfLikelihoodUsed;
    bool _parallelized;

    int _minInliers;
    int _iterations;
    int _pnpRefineIterations;
    double _pnpReprojError;
    int _pnpFlags;
};
