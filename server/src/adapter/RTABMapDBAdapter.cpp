#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UProcessInfo.h>
#include <rtabmap/utilite/UMath.h>

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/VisualWord.h>
#include <rtabmap/core/Features2d.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/util3d_features.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d_correspondences.h>
#include <rtabmap/core/util3d_registration.h>
#include <rtabmap/core/util3d_surface.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/core/Optimizer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <sqlite3.h>

#include "adapter/RTABMapDBAdapter.h"
#include "data/Labels.h"
#include "data/Signatures.h"
#include "data/Words.h"
#include "util/Time.h"

bool RTABMapDBAdapter::readData(const std::vector<std::string> &dbPaths, Words &words, Signatures &signatures, Labels &labels)
{
    int nextMemSigId = 1; // the ID we assign to next signature we put in memory
    int nextMemWordId = 1; // the ID we assign to next visual word we put in memory

    // Read data from databases
    std::map< int, std::list< std::unique_ptr<rtabmap::VisualWord> > > allWordsMap;
    std::list< std::unique_ptr<Signature> > allSignatures;
    std::list< std::unique_ptr<Label> > allLabels;
    int dbId = 0;
    for (const auto &dbPath : dbPaths)
    {
        auto dbSignatures = readSignatures(dbPath, dbId);
        std::move(dbSignatures.begin(), dbSignatures.end(), std::back_inserter(allSignatures));

        auto dbWords = readWords(dbPath, dbId, dbSignatures);
        allWordsMap.insert(std::make_pair(dbId, std::move(dbWords)));

        auto dbLabels = readLabels(dbPath, dbId, dbSignatures);
        std::move(dbLabels.begin(), dbLabels.end(), std::back_inserter(allLabels));

        dbId++;
    }

    // merge data from all databases
    auto mergeWordsIdMap = getMergeWordsIdMap(allWordsMap);
    auto mergeSignaturesIdMap = getMergeSignaturesIdMap(allSignatures);

    auto mergedSignatures = mergeSignatures(std::move(allSignatures), mergeSignaturesIdMap, mergeWordsIdMap);
    signatures.putSignatures(std::move(mergedSignatures));

    auto mergedWords = mergeWords(std::move(allWordsMap), mergeWordsIdMap, mergeSignaturesIdMap);
    words.putWords(std::move(mergedWords));

    labels.putLabels(std::move(allLabels));

    return true;
}

std::list< std::unique_ptr<Signature> > RTABMapDBAdapter::readSignatures(const std::string &dbPath, int dbId)
{
    std::list< std::unique_ptr<Signature> > signatures;

    // get optimized poses of signatures
    UDEBUG("Optimize poses of signatures...");
    std::map<int, rtabmap::Transform> optimizedPoseMap = getOptimizedPoseMap(dbPath);

    rtabmap::DBDriver *dbDriver = rtabmap::DBDriver::create();
    if (!dbDriver->openConnection(dbPath))
    {
        UDEBUG("Connecting to database %s, path is invalid!", dbPath.c_str());
        return signatures;
    }

    // Read signatures from database
    UDEBUG("Read signatures from database...");
    std::list<rtabmap::Signature *> rtabmapSignatures;
    std::set<int> sigIds;
    dbDriver->getAllNodeIds(sigIds, true);
    dbDriver->loadSignatures(std::list<int>(sigIds.begin(), sigIds.end()), rtabmapSignatures);
    dbDriver->loadNodeData(rtabmapSignatures);
    for (auto &signature : rtabmapSignatures)
    {
        int id = signature->id();
        auto iter = optimizedPoseMap.find(id);
        assert(iter != optimizedPoseMap.end());

        signature->setPose(iter->second);
        if (!signature->sensorData().imageCompressed().empty())
        {
            signature->sensorData().uncompressData();
        }
        int mapId = signature->mapId();
        const rtabmap::Transform &pose = signature->getPose();
        rtabmap::SensorData &sensorData = signature->sensorData();
        const auto &words = signature->getWords();
        const auto &words3 = signature->getWords3();
        signatures.emplace_back(std::unique_ptr<Signature>(new Signature(id, mapId, dbId, pose, std::move(sensorData), words, words3)));
            
        delete signature;
        signature = nullptr;
    }

    UDEBUG("Closing database \"%s\"...", dbDriver->getUrl().c_str());
    dbDriver->closeConnection();
    dbDriver->join();
    delete dbDriver;
    dbDriver = nullptr;

    return signatures;
}

std::list< std::unique_ptr<rtabmap::VisualWord> > RTABMapDBAdapter::readWords(const std::string &dbPath, int dbId, const std::list< std::unique_ptr<Signature> > &signatures)
{
    std::list< std::unique_ptr<rtabmap::VisualWord> > allWords;

    rtabmap::DBDriver *dbDriver = rtabmap::DBDriver::create();
    if (!dbDriver->openConnection(dbPath))
    {
        UDEBUG("Connecting to database %s, path is invalid!", dbPath.c_str());
        return allWords;
    }

    // Read words from database
    UDEBUG("Read words from database...");
    std::set<int> wordIds;
    for (const auto &signature : signatures)
    {
        const auto &words = signature->getWords();
        std::list<int> keys = uUniqueKeys(words);
        wordIds.insert(keys.begin(), keys.end());
    }
    std::list<rtabmap::VisualWord *> allDbWords;
    dbDriver->loadWords(wordIds, allDbWords);
    for (auto &word : allDbWords)
    {
        allWords.emplace_back(std::unique_ptr<rtabmap::VisualWord>(word));
        word = nullptr;
    }

    UDEBUG("Closing database \"%s\"...", dbDriver->getUrl().c_str());
    dbDriver->closeConnection();
    dbDriver->join();
    delete dbDriver;
    dbDriver = nullptr;

    return allWords;
}

std::list< std::unique_ptr<Label> > RTABMapDBAdapter::readLabels(const std::string &dbPath, int dbId, const std::list< std::unique_ptr<Signature> > &signatures)
{
    std::list< std::unique_ptr<Label> > labels;
    sqlite3 *db = nullptr;
    sqlite3_stmt *stmt = nullptr;
    int rc;

    rc = sqlite3_open(dbPath.c_str(), &db);
    if (rc != SQLITE_OK)
    {
        UERROR("Could not open database %s", sqlite3_errmsg(db));
        sqlite3_close(db);
        return labels;
    }

    std::string sql = "SELECT * from Labels";
    rc = sqlite3_prepare(db, sql.c_str(), -1, &stmt, nullptr);
    if (rc == SQLITE_OK)
    {
        while (sqlite3_step(stmt) == SQLITE_ROW)
        {
            std::string name(reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0)));
            int imageId = sqlite3_column_int(stmt, 1);
            int x = sqlite3_column_int(stmt, 2);
            int y = sqlite3_column_int(stmt, 3);
            pcl::PointXYZ pWorld;
            if (getPoint3World(signatures, dbId, imageId, x, y, pWorld))
            {
                labels.emplace_back(std::unique_ptr<Label>(new Label(dbId, imageId, cv::Point2f(x, y), cv::Point3f(pWorld.x, pWorld.y, pWorld.z), name)));
                UINFO("Read point (%lf,%lf,%lf) with label %s in database %s", pWorld.x, pWorld.y, pWorld.z, name.c_str(), dbPath.c_str());
            }
        }
    }
    else
    {
        UWARN("Could not read database %s: %s", dbPath.c_str(), sqlite3_errmsg(db));
    }

    sqlite3_finalize(stmt);
    sqlite3_close(db);

    return labels;
}

std::map<int, rtabmap::Transform> RTABMapDBAdapter::getOptimizedPoseMap(const std::string &dbPath)
{
    rtabmap::Memory memory;
    memory.init(dbPath);

    std::map<int, rtabmap::Transform> optimizedPoseMap;
    if (memory.getLastWorkingSignature())
    {
        // Get all IDs linked to last signature (including those in Long-Term Memory)
        std::map<int, int> ids = memory.getNeighborsId(memory.getLastWorkingSignature()->id(), 0, -1);

        // Get all metric constraints (the graph)
        std::map<int, rtabmap::Transform> poses;
        std::multimap<int, rtabmap::Link> links;
        memory.getMetricConstraints(uKeysSet(ids), poses, links, true);

        // Optimize the graph
        rtabmap::Optimizer *graphOptimizer = rtabmap::Optimizer::create(rtabmap::Optimizer::kTypeTORO);
        optimizedPoseMap = graphOptimizer->optimize(poses.begin()->first, poses, links);
        delete graphOptimizer;
    }

    return optimizedPoseMap;
}

bool RTABMapDBAdapter::getPoint3World(const std::list< std::unique_ptr<Signature> > &signatures, int dbId, int imageId, int x, int y, pcl::PointXYZ &pWorld)
{
    UDEBUG("");
    Signature *signature = nullptr;
    // TODO: Use map of map for both signature and words
    for (const auto &signature : signatures)
    {
        if (signature->getDbId() == dbId && signature->getId() == imageId)
        {
            break;
        }
    }
    assert(signature != nullptr);

    const rtabmap::SensorData &data = signature->getSensorData();
    const rtabmap::CameraModel &cm = data.cameraModels()[0];
    bool smoothing = false;
    assert(!data.depthRaw().empty());
    pcl::PointXYZ pLocal = rtabmap::util3d::projectDepthTo3D(data.depthRaw(), x, y, cm.cx(), cm.cy(), cm.fx(), cm.fy(), smoothing);
    if (std::isnan(pLocal.x) || std::isnan(pLocal.y) || std::isnan(pLocal.z))
    {
        UWARN("Depth value not valid");
        return false;
    }
    rtabmap::Transform poseWorld = signature->getPose();
    if (poseWorld.isNull())
    {
        UWARN("Image pose is Null");
        return false;
    }
    poseWorld = poseWorld * cm.localTransform();
    pWorld = rtabmap::util3d::transformPoint(pLocal, poseWorld);
    return true;
}

std::map<std::pair<int, int>, int> RTABMapDBAdapter::getMergeWordsIdMap(const std::map< int, std::list< std::unique_ptr<rtabmap::VisualWord> > > &wordsMap)
{
    std::map<std::pair<int, int>, int> mergeWordsIdMap;
    int nextWordId = 1;
    for (const auto &words : wordsMap)
    {
        int dbId = words.first;
        for (const auto &word : words.second)
        {
            int wordId = word->id();
            mergeWordsIdMap.insert(std::make_pair(std::make_pair(dbId, wordId), nextWordId));
            nextWordId++;
        }
    }
    return mergeWordsIdMap;
}

std::map<std::pair<int, int>, int> RTABMapDBAdapter::getMergeSignaturesIdMap(const std::list< std::unique_ptr<Signature> > &signatures)
{
    std::map<std::pair<int, int>, int> mergeSignaturesIdMap;
    int nextSignatureId = 1;
    for (const auto &signature : signatures)
    {
        int dbId = signature->getDbId();
        int signatureId = signature->getId();
        mergeSignaturesIdMap.insert(std::pair<std::pair<int, int>, int>(std::pair<int, int>(dbId, signatureId), nextSignatureId));
        nextSignatureId++;
    }
    return mergeSignaturesIdMap;
}

std::list< std::unique_ptr<rtabmap::VisualWord> > RTABMapDBAdapter::mergeWords(std::map< int, std::list< std::unique_ptr<rtabmap::VisualWord> > > &&wordsMap, const std::map<std::pair<int, int>, int> &mergeWordsIdMap, const std::map<std::pair<int, int>, int> &mergeSignaturesIdMap)
{
    std::list< std::unique_ptr<rtabmap::VisualWord> > mergedWords;

    for (const auto &words : wordsMap)
    {
        int dbId = words.first;
        for (const auto &word : words.second)
        {
            int wordId = word->id();
            auto wordIdIter = mergeWordsIdMap.find(std::make_pair(dbId, wordId));
            // TODO update reference signatures
            if (wordIdIter != mergeWordsIdMap.end())
            {
                int newId = wordIdIter->second;
                const cv::Mat &descriptor = word->getDescriptor();
                mergedWords.emplace_back(std::unique_ptr<rtabmap::VisualWord>(new rtabmap::VisualWord(newId, descriptor)));
            }
        }
    }
    return mergedWords;
}

std::list< std::unique_ptr<Signature> > RTABMapDBAdapter::mergeSignatures(std::list< std::unique_ptr<Signature> > &&signatures, const std::map<std::pair<int, int>, int> &mergeSignaturesIdMap, const std::map<std::pair<int, int>, int> &mergeWordsIdMap)
{
    std::list< std::unique_ptr<Signature> > mergedSignatures;

    for (auto &signature : signatures)
    {
        int dbId = signature->getDbId();
        int signatureId = signature->getId();
        auto signatureIdIter = mergeSignaturesIdMap.find(std::make_pair(dbId, signatureId));
        assert(signatureIdIter != mergeSignaturesIdMap.end());

        int newId = signatureIdIter->second;
        int mapId = signature->getMapId();
        const rtabmap::Transform &pose = signature->getPose();
        const rtabmap::SensorData &sensorData = signature->getSensorData();

        std::multimap<int, cv::KeyPoint> words;
        for (const auto &word : signature->getWords())
        {
            auto wordIdIter = mergeWordsIdMap.find(std::make_pair(dbId, word.first));
            assert(wordIdIter != mergeWordsIdMap.end());
            words.insert(std::make_pair(wordIdIter->second, word.second));
        }

        std::multimap<int, cv::Point3f> words3;
        for (const auto &word3 : signature->getWords3())
        {
            auto wordIdIter = mergeWordsIdMap.find(std::make_pair(dbId, word3.first));
            assert(wordIdIter != mergeWordsIdMap.end());
            words3.insert(std::make_pair(wordIdIter->second, word3.second));
        }

        mergedSignatures.emplace_back(std::unique_ptr<Signature>(new Signature(newId, mapId, dbId, pose, sensorData, std::move(words), std::move(words3))));
    }

    return mergedSignatures;
}
