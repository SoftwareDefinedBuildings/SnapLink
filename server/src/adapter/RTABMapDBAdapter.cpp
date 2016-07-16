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
#include "util/Time.h"
#include "data/Words.h"
#include "data/Signatures.h"
#include "data/Labels.h"

RTABMapDBAdapter::RTABMapDBAdapter()
{
}

RTABMapDBAdapter::~RTABMapDBAdapter()
{
}

static bool RTABMapDBAdapter::readData(std::vector<std::string> &dbPaths, Words *words, Signatures *signatures, Labels *labels)
{
    int nextMemSigId = 1; // the ID we assign to next signature we put in memory
    int nextMemWordId = 1; // the ID we assign to next visual word we put in memory

    // Read data from databases
    std::map< int, std::map<int, rtabmap::Signature *> > dbSignatureMaps;
    for (int dbId = 0; dbId < dbPaths.size(); dbId++)
    {
        std::string &dbPath = dbPaths.at(dbId);

        if (signatures != NULL)
        {
            // get optimized poses of signatures
            UDEBUG("Optimize poses of signatures...");
            std::map<int, Transform> optimizedPoses = getOptimizePoses(dbPath);

            // read data from databases
            rtabmap::DBDriver *dbDriver = rtabmap::DBDriver::create();
            if (!dbDriver->openConnection(dbPath))
            {
                UDEBUG("Connecting to database %s, path is invalid!", dbPath.c_str());
                return false;
            }
            UDEBUG("Connecting to database %s done!", dbPath.c_str());

            // Read signatures from database
            UDEBUG("Read signatures from database...");
            std::list<rtabmap::Signature *> signatures;
            std::set<int> sigIds;
            dbDriver->getAllNodeIds(sigIds, true);
            dbDriver->loadSignatures(std::list<int>(sigIds.begin(), sigIds.end()), signatures);
            dbDriver->loadNodeData(signatures);
            std::map<int, rtabmap::Signature *> signatureMap;
            for (std::list<rtabmap::Signature *>::iterator iter = signatures.begin(); iter != signatures.end(); iter++)
            {
                rtabmap::Signature *signature = *iter;
                int signatureId = signature->id();
                std::map<int, Transform>::const_iterator jter = optimizedPoses.find(signatureId);
                if (jter != optimizedPoses.end())
                {
                    signature->setPose(jter->second);
                    signatureMap.insert(std::pair<int, rtabmap::Signature *>(signatureId, signature));
                }
                else
                {
                    UWARN("Cannot find optimized pose for signature %d in database %d", signatureId, dbId);
                }
            }
        }

        if (signatures != NULL)
        {
            // Read words from database
            UDEBUG("Read words from database...");
            std::set<int> wordIds;
            for (std::list<rtabmap::Signature *>::const_iterator iter = signatures.begin(); iter != signatures.end(); ++iter)
            {
                const std::multimap<int, cv::KeyPoint> &words = (*iter)->getWords();
                std::list<int> keys = uUniqueKeys(words);
                wordIds.insert(keys.begin(), keys.end());
            }
            std::list<rtabmap::VisualWord *> words;
            dbDriver->loadWords(wordIds, words);
        }

        UDEBUG("Closing database \"%s\"...", dbDriver->getUrl().c_str());
        dbDriver->closeConnection();
        dbDriver->join();
        delete dbDriver;
        dbDriver = NULL;
        UDEBUG("Closing database, done!");

        // Read labels from database before updating signature ids
        if (labels != NULL)
        {
            UDEBUG("Read labels from database...");
            std::list<Label *> memLabels = readLabels(dbId, db);
            labels->addLabels(memLabels);
        }


        dbSignatureMaps.insert(std::pair< int, std::map<int, rtabmap::Signature *> >(dbId, signatureMap));
    }

    // Re-assign signature Id and word Id
    {
        // Update signature ids and words in memory
        UDEBUG("Update signatures ids and words in memory...");
        dbSignatureMap = _signatureMaps.at(dbId);
        std::map<int, rtabmap::Signature *> memSignatureMap; // mem sig map with updated pose and ids
        for (std::map<int, rtabmap::Signature *>::const_iterator iter = dbSignatureMap.begin(); iter != dbSignatureMap.end(); ++iter)
        {
            const rtabmap::Signature *dbSig = iter->second;
            std::map<int, int>::const_iterator idIter = sigIdMap.find(dbSig->id());
            if (idIter == sigIdMap.end())
            {
                UFATAL("");
            }
            rtabmap::SensorData sensorData = dbSig->sensorData();
            sensorData.setId(idIter->second);
            // TODO use our own memory signature definition
            rtabmap::Signature *memSig = new rtabmap::Signature(idIter->second, dbSig->mapId(), dbSig->getWeight(), dbSig->getStamp(), dbSig->getLabel(), dbSig->getPose(), dbSig->getGroundTruthPose(), sensorData);

            // Update signatures words
            const std::multimap<int, cv::KeyPoint> &dbSigWords = dbSig->getWords();
            std::multimap<int, cv::KeyPoint> memSigWords;
            for (std::multimap<int, cv::KeyPoint>::const_iterator jter = dbSigWords.begin(); jter != dbSigWords.end(); jter++)
            {
                std::map<int, int>::iterator kter = wordIdMap.find(jter->first);
                if (kter == wordIdMap.end())
                {
                    UFATAL("");
                }
                memSigWords.insert(std::pair<int, cv::KeyPoint>(kter->second, jter->second));
            }
            memSig->setWords(memSigWords); // it's pass by value internally
            const std::multimap<int, cv::Point3f> &dbSigWords3 = dbSig->getWords3();
            std::multimap<int, cv::Point3f> memSigWords3;
            for (std::multimap<int, cv::Point3f>::const_iterator jter = dbSigWords3.begin(); jter != dbSigWords3.end(); jter++)
            {
                std::map<int, int>::iterator kter = wordIdMap.find(jter->first);
                if (kter == wordIdMap.end())
                {
                    UFATAL("");
                }
                memSigWords3.insert(std::pair<int, cv::Point3f>(kter->second, jter->second));
            }
            memSig->setWords3(memSigWords3); // it's pass by value internally

            memSignatureMap.insert(std::pair<int, rtabmap::Signature *>(memSig->id(), memSig));

            delete dbSig;
        }
        _signatureMaps.at(dbId) = memSignatureMap;
        std::list<Signature *> memSignatures;
        for (std::map<int, rtabmap::Signature *>::iterator iter = memSignatureMap.begin(); iter != memSignatureMap.end(); iter++)
        {
            Signature *memSig = new Signature(iter->second->id(), iter->second->mapId(), dbId, iter->second->getPose(),
                                              iter->second->sensorData(), iter->second->getWords(), iter->second->getWords3());
            memSignatures.push_back(memSig);
            delete iter->second;
        }
        _signatures->addSignatures(memSignatures);
        UDEBUG("Loading signatures done! (%d loaded)", int(memSignatureMap.size()));

        // Add words to memory
        UDEBUG("Add words to memory...");
        std::list<rtabmap::VisualWord *> memWords;
        for (std::list<rtabmap::VisualWord *>::iterator iter = words.begin(); iter != words.end(); ++iter)
        {
            rtabmap::VisualWord *dbVW = *iter;
            std::map<int, int>::const_iterator idIter = wordIdMap.find(dbVW->id());
            if (idIter == wordIdMap.end())
            {
                UFATAL("");
            }
            rtabmap::VisualWord *memVW = new rtabmap::VisualWord(idIter->second, dbVW->getDescriptor());
            // TODO add signature references to mem word
            memWords.push_back(memVW);
            delete dbVW;
        }
        _words->addWords(memWords);
        UDEBUG("%d words loaded!", words.size());

        // UDEBUG("Adding word references...");
        // // Enable loaded signatures
        // for (std::map<int, rtabmap::Signature *>::const_iterator iter = signatureMap.begin(); iter != signatureMap.end(); ++iter)
        // {
        //     rtabmap::Signature *sig = iter->second;
        //     const std::multimap<int, cv::KeyPoint> &words = sig->getWords();
        //     if (words.size())
        //     {
        //         UDEBUG("node=%d, word references=%d", sig->id(), words.size());
        //         for (std::multimap<int, cv::KeyPoint>::const_iterator jter = words.begin(); jter != words.end(); ++jter)
        //         {
        //             _words->addWordRef(jter->first, iter->first);
        //         }
        //         sig->setEnabled(true);
        //     }
        // }
        // UDEBUG("Adding word references, done! (%d)", _words->getTotalActiveReferences());
        //
        // if (_words->getUnusedWordsSize())
        // {
        //     UWARN("_words->getUnusedWordsSize() must be empty... size=%d", _words->getUnusedWordsSize());
        // }
        //UDEBUG("Total word references added = %d", _words->getTotalActiveReferences());
    }
}

bool RTABMapDBAdapter::init(const rtabmap::ParametersMap &parameters)
{
    UDEBUG("");

    this->parseParameters(parameters);

    return true;
}

void RTABMapDBAdapter::parseParameters(const rtabmap::ParametersMap &parameters)
{
    uInsert(parameters_, parameters);
}

std::list<Label *> RTABMapDBAdapter::readLabels(int dbId, std::string dbPath) const
{
    std::list<Label *> labels;
    sqlite3 *db = NULL;
    sqlite3_stmt *stmt = NULL;
    int rc;

    rc = sqlite3_open(dbPath.c_str(), &db);
    if (rc != SQLITE_OK)
    {
        UERROR("Could not open database %s", sqlite3_errmsg(db));
        sqlite3_close(db);
        return labels;
    }

    std::string sql = "SELECT * from Labels";
    rc = sqlite3_prepare(db, sql.c_str(), -1, &stmt, NULL);
    if (rc == SQLITE_OK)
    {
        while (sqlite3_step(stmt) == SQLITE_ROW)
        {
            std::string name(reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0)));
            int imageId = sqlite3_column_int(stmt, 1);
            int x = sqlite3_column_int(stmt, 2);
            int y = sqlite3_column_int(stmt, 3);
            pcl::PointXYZ pWorld;
            if (getPoint3World(dbId, imageId, x, y, pWorld))
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

bool RTABMapDBAdapter::getPoint3World(int dbId, int imageId, int x, int y, pcl::PointXYZ &pWorld) const
{
    UDEBUG("");
    const rtabmap::Signature *s = _signatureMaps.at(dbId).at(imageId);
    if (s == NULL)
    {
        UWARN("Signature %d does not exist", imageId);
        return false;
    }
    const rtabmap::SensorData &data = s->sensorData();
    const rtabmap::CameraModel &cm = data.cameraModels()[0];
    bool smoothing = false;
    pcl::PointXYZ pLocal = rtabmap::util3d::projectDepthTo3D(data.depthRaw(), x, y, cm.cx(), cm.cy(), cm.fx(), cm.fy(), smoothing);
    if (std::isnan(pLocal.x) || std::isnan(pLocal.y) || std::isnan(pLocal.z))
    {
        UWARN("Depth value not valid");
        return false;
    }
    rtabmap::Transform poseWorld = s->getPose();
    if (poseWorld.isNull())
    {
        UWARN("Image pose is Null");
        return false;
    }
    poseWorld = poseWorld * cm.localTransform();
    pWorld = rtabmap::util3d::transformPoint(pLocal, poseWorld);
    return true;
}

static std::map<int, Transform> RTABMapDBAdapter::getOptimizedPoses(std::string dbPath)
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
