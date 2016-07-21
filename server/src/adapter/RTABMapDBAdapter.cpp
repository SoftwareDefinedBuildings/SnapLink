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
    std::map< int, std::list<rtabmap::VisualWord *> > allWordsMap;
    std::list<Signature *> allSignatures;
    std::list<Label *> allLabels;
    for (int dbId = 0; dbId < dbPaths.size(); dbId++)
    {
        const std::string &dbPath = dbPaths.at(dbId);

        std::list<Signature *> dbSignatures = readSignatures(dbPath, dbId);
        allSignatures.insert(allSignatures.end(), dbSignatures.begin(), dbSignatures.end());

        std::list<rtabmap::VisualWord *> dbWords = readWords(dbPath, dbId, dbSignatures);
        allWordsMap.insert(std::pair< int, std::list<rtabmap::VisualWord *> >(dbId, dbWords));

        UDEBUG("Read labels from database...");
        std::list<Label *> dbLabels = readLabels(dbPath, dbId, dbSignatures);
        allLabels.insert(allLabels.end(), dbLabels.begin(), dbLabels.end());
    }

    // merge data from all databases
    std::map<std::pair<int, int>, int> mergeWordsIdMap = getMergeWordsIdMap(allWordsMap);
    std::map<std::pair<int, int>, int> mergeSignaturesIdMap = getMergeSignaturesIdMap(allSignatures);

    std::list<Signature *> mergedSignatures = mergeSignatures(allSignatures, mergeSignaturesIdMap, mergeWordsIdMap);
    signatures.addSignatures(mergedSignatures);

    std::list<rtabmap::VisualWord *> mergedWords = mergeWords(allWordsMap, mergeWordsIdMap, mergeSignaturesIdMap);
    words.addWords(mergedWords);

    labels.addLabels(allLabels);

    return true;
}

std::list<Signature *> RTABMapDBAdapter::readSignatures(const std::string &dbPath, int dbId)
{
    std::list<Signature *> signatures;

    // get optimized poses of signatures
    UDEBUG("Optimize poses of signatures...");
    std::map<int, rtabmap::Transform> optimizedPoses = getOptimizedPoses(dbPath);

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
    for (auto & iter : rtabmapSignatures)
    {
        rtabmap::Signature *rtabmapSignature = iter;
        int signatureId = rtabmapSignature->id();
        std::map<int, rtabmap::Transform>::const_iterator jter = optimizedPoses.find(signatureId);
        if (jter != optimizedPoses.end())
        {
            rtabmapSignature->setPose(jter->second);
            if (!rtabmapSignature->sensorData().imageCompressed().empty())
            {
                rtabmapSignature->sensorData().uncompressData();
            }
            Signature *signature = convertSignature(*rtabmapSignature, dbId);
            signatures.push_back(signature);
        }
        else
        {
            UWARN("Cannot find optimized pose for signature %d in database %d", signatureId, dbId);
        }
        delete rtabmapSignature;
        rtabmapSignature = nullptr;
        iter = nullptr;
    }

    UDEBUG("Closing database \"%s\"...", dbDriver->getUrl().c_str());
    dbDriver->closeConnection();
    dbDriver->join();
    delete dbDriver;
    dbDriver = nullptr;

    return signatures;
}

std::list<rtabmap::VisualWord *> RTABMapDBAdapter::readWords(const std::string &dbPath, int dbId, std::list<Signature *> &signatures)
{
    std::list<rtabmap::VisualWord *> words;

    rtabmap::DBDriver *dbDriver = rtabmap::DBDriver::create();
    if (!dbDriver->openConnection(dbPath))
    {
        UDEBUG("Connecting to database %s, path is invalid!", dbPath.c_str());
        return words;
    }

    // Read words from database
    UDEBUG("Read words from database...");
    std::set<int> wordIds;
    for (std::list<Signature *>::const_iterator iter = signatures.begin(); iter != signatures.end(); ++iter)
    {
        const std::multimap<int, cv::KeyPoint> &words = (*iter)->getWords();
        std::list<int> keys = uUniqueKeys(words);
        wordIds.insert(keys.begin(), keys.end());
    }
    dbDriver->loadWords(wordIds, words);

    UDEBUG("Closing database \"%s\"...", dbDriver->getUrl().c_str());
    dbDriver->closeConnection();
    dbDriver->join();
    delete dbDriver;
    dbDriver = nullptr;

    return words;
}

std::list<Label *> RTABMapDBAdapter::readLabels(const std::string &dbPath, int dbId, const std::list<Signature *> &signatures)
{
    std::list<Label *> labels;
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
                labels.push_back(new Label(dbId, imageId, cv::Point2f(x, y), cv::Point3f(pWorld.x, pWorld.y, pWorld.z), name));
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

std::map<int, rtabmap::Transform> RTABMapDBAdapter::getOptimizedPoses(const std::string &dbPath)
{
    rtabmap::Memory memory;
    memory.init(dbPath);

    std::map<int, rtabmap::Transform> optimizedPoses;
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
        optimizedPoses = graphOptimizer->optimize(poses.begin()->first, poses, links);
        delete graphOptimizer;
    }

    return optimizedPoses;
}

Signature *RTABMapDBAdapter::convertSignature(const rtabmap::Signature &signature, int dbId)
{
    auto newSignature = new Signature(signature.id(), signature.mapId(), dbId, signature.getPose(), signature.sensorData(), signature.getWords(), signature.getWords3());
    return newSignature;
}

bool RTABMapDBAdapter::getPoint3World(const std::list<Signature *> &signatures, int dbId, int imageId, int x, int y, pcl::PointXYZ &pWorld)
{
    UDEBUG("");
    Signature *signature = nullptr;
    // TODO: Use map of map for both signature and words
    for (auto iter : signatures)
    {
        signature = iter;
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

std::map<std::pair<int, int>, int> RTABMapDBAdapter::getMergeWordsIdMap(const std::map< int, std::list<rtabmap::VisualWord *> > &words)
{
    std::map<std::pair<int, int>, int> mergeWordsIdMap;
    int nextWordId = 1;
    for (const auto & word : words)
    {
        for (auto jter = word.second.begin(); jter != word.second.end(); jter++)
        {
            int dbId = word.first;
            int wordId = (*jter)->id();
            mergeWordsIdMap.insert(std::pair<std::pair<int, int>, int>(std::pair<int, int>(dbId, wordId), nextWordId));
            nextWordId++;
        }
    }
    return mergeWordsIdMap;
}

std::map<std::pair<int, int>, int> RTABMapDBAdapter::getMergeSignaturesIdMap(const std::list<Signature *> &signatures)
{
    std::map<std::pair<int, int>, int> mergeSignaturesIdMap;
    int nextSignatureId = 1;
    for (auto signature : signatures)
    {
        int dbId = signature->getDbId();
        int signatureId = signature->getId();
        mergeSignaturesIdMap.insert(std::pair<std::pair<int, int>, int>(std::pair<int, int>(dbId, signatureId), nextSignatureId));
        nextSignatureId++;
    }
    return mergeSignaturesIdMap;
}

std::list<rtabmap::VisualWord *> RTABMapDBAdapter::mergeWords(const std::map< int, std::list<rtabmap::VisualWord *> > &words, const std::map<std::pair<int, int>, int> &mergeWordsIdMap, const std::map<std::pair<int, int>, int> &mergeSignaturesIdMap)
{
    std::list<rtabmap::VisualWord *> mergedWords;

    for (const auto & wordsIter : words)
    {
        for (auto wordIter = wordsIter.second.begin(); wordIter != wordsIter.second.end(); wordIter++)
        {
            rtabmap::VisualWord *word = *wordIter;
            int dbId = wordsIter.first;
            int wordId = word->id();
            auto wordIdIter = mergeWordsIdMap.find(std::pair<int, int>(dbId, wordId));
            // TODO update reference signatures
            if (wordIdIter != mergeWordsIdMap.end())
            {
                mergedWords.push_back(new rtabmap::VisualWord(wordIdIter->second, word->getDescriptor()));
            }
            delete word;
            word = nullptr;
        }
    }
    return mergedWords;
}

std::list<Signature *> RTABMapDBAdapter::mergeSignatures(const std::list<Signature *> &signatures, const std::map<std::pair<int, int>, int> &mergeSignaturesIdMap, const std::map<std::pair<int, int>, int> &mergeWordsIdMap)
{
    std::list<Signature *> mergedSignatures;

    for (auto signature : signatures)
    {
        int dbId = signature->getDbId();
        int signatureId = signature->getId();
        auto signatureIdIter = mergeSignaturesIdMap.find(std::pair<int, int>(dbId, signatureId));
        if (signatureIdIter != mergeSignaturesIdMap.end())
        {
            std::multimap<int, cv::KeyPoint> words;
            for (const auto & wordIter : signature->getWords())
            {
                auto wordIdIter = mergeWordsIdMap.find(std::pair<int, int>(dbId, wordIter.first));
                assert(wordIdIter != mergeWordsIdMap.end());
                words.insert(std::pair<int, cv::KeyPoint>(wordIdIter->second, wordIter.second));
            }
            std::multimap<int, cv::Point3f> words3;
            for (const auto & word3Iter : signature->getWords3())
            {
                auto wordIdIter = mergeWordsIdMap.find(std::pair<int, int>(dbId, word3Iter.first));
                assert(wordIdIter != mergeWordsIdMap.end());
                words3.insert(std::pair<int, cv::Point3f>(wordIdIter->second, word3Iter.second));
            }
            mergedSignatures.push_back(new Signature(signatureIdIter->second, signature->getMapId(), dbId, signature->getPose(), signature->getSensorData(), words, words3));
        }
        delete signature;
        signature = nullptr;
    }
    return mergedSignatures;
}
