#pragma once

#include "rtabmap/utilite/UEventsHandler.h"
#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/SensorData.h"
#include "rtabmap/core/Link.h"
#include "rtabmap/core/Features2d.h"
#include <typeinfo>
#include <list>
#include <map>
#include <set>
#include "rtabmap/utilite/UStl.h"
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
class Stereo;

class MemoryLoc
{
public:
    static const int kIdStart;
    static const int kIdVirtual;
    static const int kIdInvalid;

public:
    MemoryLoc(const ParametersMap &parameters = ParametersMap());
    virtual ~MemoryLoc();

    virtual void parseParameters(const ParametersMap &parameters);
    virtual const ParametersMap &getParameters() const
    {
        return parameters_;
    }
    bool update(const SensorData &data,
                Statistics *stats = 0);
    bool update(const SensorData &data,
                const Transform &pose,
                const cv::Mat &covariance,
                Statistics *stats = 0);
    bool init(const std::string &dbUrl,
              bool dbOverwritten = false,
              const ParametersMap &parameters = ParametersMap(),
              bool postInitClosingEvents = false);
    void close(bool databaseSaved = true, bool postInitClosingEvents = false);
    std::map<int, float> computeLikelihood(const Signature *signature,
                                           const std::list<int> &ids);
    int incrementMapId(std::map<int, int> *reducedIds = 0);
    void updateAge(int signatureId);

    std::list<int> forget(const std::set<int> &ignoredIds = std::set<int>());
    std::set<int> reactivateSignatures(const std::list<int> &ids, unsigned int maxLoaded, double &timeDbAccess);

    int cleanup();
    void emptyTrash();
    void joinTrashThread();
    bool addLink(const Link &link);
    void updateLink(int fromId, int toId, const Transform &transform, float rotVariance, float transVariance);
    void updateLink(int fromId, int toId, const Transform &transform, const cv::Mat &covariance);
    void removeAllVirtualLinks();
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
        const std::map<int, Transform> &optimizedPoses,
        int maxGraphDepth) const;
    void deleteLocation(int locationId, std::list<int> *deletedWords = 0);
    void removeLink(int idA, int idB);
    void removeRawData(int id, bool image = true, bool scan = true, bool userData = true);

    //getters
    const std::map<int, double> &getWorkingMem() const
    {
        return _workingMem;
    }
    const std::set<int> &getStMem() const
    {
        return _stMem;
    }
    int getMaxStMemSize() const
    {
        return _maxStMemSize;
    }
    std::map<int, Link> getNeighborLinks(int signatureId,
                                         bool lookInDatabase = false) const;
    std::map<int, Link> getLoopClosureLinks(int signatureId,
                                            bool lookInDatabase = false) const;
    std::map<int, Link> getLinks(int signatureId,
                                 bool lookInDatabase = false) const;
    std::multimap<int, Link> getAllLinks(bool lookInDatabase, bool ignoreNullLinks = true) const;
    bool isBinDataKept() const
    {
        return _binDataKept;
    }
    float getSimilarityThreshold() const
    {
        return _similarityThreshold;
    }
    std::map<int, int> getWeights() const;
    int getLastSignatureId() const;
    const Signature *getLastWorkingSignature() const;
    int getSignatureIdByLabel(const std::string &label, bool lookInDatabase = true) const;
    bool labelSignature(int id, const std::string &label);
    std::map<int, std::string> getAllLabels() const;
    bool setUserData(int id, const cv::Mat &data);
    int getDatabaseMemoryUsed() const; // in bytes
    std::string getDatabaseVersion() const;
    double getDbSavingTime() const;
    Transform getOdomPose(int signatureId, bool lookInDatabase = false) const;
    Transform getGroundTruthPose(int signatureId, bool lookInDatabase = false) const;
    bool getNodeInfo(int signatureId,
                     Transform &odomPose,
                     int &mapId,
                     int &weight,
                     std::string &label,
                     double &stamp,
                     Transform &groundTruth,
                     bool lookInDatabase = false) const;
    cv::Mat getImageCompressed(int signatureId) const;
    SensorData getNodeData(int nodeId, bool uncompressedData = false, bool keepLoadedDataInMemory = true);
    void getNodeWords(int nodeId,
                      std::multimap<int, cv::KeyPoint> &words,
                      std::multimap<int, cv::Point3f> &words3);
    SensorData getSignatureDataConst(int locationId) const;
    std::set<int> getAllSignatureIds() const;
    bool memoryChanged() const
    {
        return _memoryChanged;
    }
    bool isIncremental() const
    {
        return _incrementalMemory;
    }
    const Signature *getSignature(int id) const;
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
    const Feature2D *getFeature2D() const
    {
        return _feature2D;
    }
    bool isGraphReduced() const
    {
        return _reduceGraph;
    }

    void dumpMemoryTree(const char *fileNameTree) const;
    virtual void dumpMemory(std::string directory) const;
    virtual void dumpSignatures(const char *fileNameSign, bool words3D) const;
    void dumpDictionary(const char *fileNameRef, const char *fileNameDesc) const;

    void generateGraph(const std::string &fileName, const std::set<int> &ids = std::set<int>());

    //keypoint stuff
    const VWDictionary *getVWDictionary() const;

    // RGB-D stuff
    void getMetricConstraints(
        const std::set<int> &ids,
        std::map<int, Transform> &poses,
        std::multimap<int, Link> &links,
        bool lookInDatabase = false);

    Transform computeTransform(int fromId, int toId, Transform guess, RegistrationInfo *info = 0);
    Transform computeIcpTransform(int fromId, int toId, Transform guess, RegistrationInfo *info = 0);
    Transform computeIcpTransformMulti(
        int newId,
        int oldId,
        const std::map<int, Transform> &poses,
        RegistrationInfo *info = 0);

    rtabmap::Transform computeGlobalVisualTransform(const std::vector<int> &oldIds, int newId) const;
    rtabmap::Transform computeGlobalVisualTransform(const std::vector<const rtabmap::Signature *> &oldSigs, const rtabmap::Signature *newSig) const;

private:
    void preUpdate();
    void addSignatureToStm(Signature *signature, const cv::Mat &covariance);
    void clear();
    void moveToTrash(Signature *s, bool keepLinkedToGraph = true, std::list<int> *deletedWords = 0);

    void moveSignatureToWMFromSTM(int id, int *reducedTo = 0);
    void addSignatureToWmFromLTM(Signature *signature);
    Signature *_getSignature(int id) const;
    std::list<Signature *> getRemovableSignatures(int count,
            const std::set<int> &ignoredIds = std::set<int>());
    int getNextId();
    void initCountId();
    void rehearsal(Signature *signature, Statistics *stats = 0);
    bool rehearsalMerge(int oldId, int newId);

    const std::map<int, Signature *> &getSignatures() const
    {
        return _signatures;
    }

    void copyData(const Signature *from, Signature *to);
    Signature *createSignature(
        const SensorData &data,
        const Transform &pose,
        Statistics *stats = 0);

    //keypoint stuff
    void disableWordsRef(int signatureId);
    void enableWordsRef(const std::list<int> &signatureIds);
    void cleanUnusedWords();
    int getNi(int signatureId) const;

protected:
    DBDriver *_dbDriver;

private:
    // parameters
    ParametersMap parameters_;
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
    Signature *_lastSignature;
    int _lastGlobalLoopClosureId;
    bool _memoryChanged; // False by default, become true only when MemoryLoc::update() is called.
    bool _linksChanged; // False by default, become true when links are modified.
    int _signaturesAdded;

    std::map<int, Signature *> _signatures; // TODO : check if a signature is already added? although it is not supposed to occur...
    std::set<int> _stMem; // id
    std::map<int, double> _workingMem; // id,age

    //Keypoint stuff
    VWDictionary *_vwd;
    Feature2D *_feature2D;
    float _badSignRatio;;
    bool _tfIdfLikelihoodUsed;
    bool _parallelized;

    Registration *_registrationPipeline;
    RegistrationIcp *_registrationIcp;

    int _minInliers;
    int _iterations;
    int _pnpRefineIterations;
    double _pnpReprojError;
    int _pnpFlags;
};
