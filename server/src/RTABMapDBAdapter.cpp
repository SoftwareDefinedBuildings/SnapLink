#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UProcessInfo.h>
#include <rtabmap/utilite/UMath.h>

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/VisualWord.h>
#include <rtabmap/core/Features2d.h>
#include <rtabmap/core/DBDriver.h>
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

#include "RTABMapDBAdapter.h"
#include "HTTPServer.h"
#include "Time.h"
#include "WordsKdTree.h"
#include "SignaturesSimple.h"
#include "LabelsSimple.h"

const int RTABMapDBAdapter::kIdStart = 0;

RTABMapDBAdapter::RTABMapDBAdapter() :
    _words(NULL),
    _signatures(NULL)
{
}

RTABMapDBAdapter::~RTABMapDBAdapter()
{
    if (_words)
    {
        delete _words;
    }
    if (_signatures)
    {
        delete _signatures;
    }
}

bool RTABMapDBAdapter::init(std::vector<std::string> &dbUrls, const rtabmap::ParametersMap &parameters)
{
    UDEBUG("");

    _words = new WordsKdTree();
    _signatures = new SignaturesSimple();
    _labels = new LabelsSimple();
    this->parseParameters(parameters);

    std::list<Label *> allLabels;
    int nextMemSigId = 1; // the ID we assign to next signature we put in memory
    int nextMemWordId = 1; // the ID we assign to next visual word we put in memory
    for (int dbId = 0; dbId < dbUrls.size(); dbId++)
    {
        std::string &dbUrl = dbUrls.at(dbId);
        rtabmap::DBDriver *dbDriver = rtabmap::DBDriver::create(parameters);
        if (!dbDriver->openConnection(dbUrl))
        {
            UDEBUG("Connecting to database %s, path is invalid!", dbUrl.c_str());
            return false;
        }
        UDEBUG("Connecting to database %s done!", dbUrl.c_str());

        // Read signatures from database
        UDEBUG("Read signatures from database...");
        std::list<rtabmap::Signature *> signatures;
        std::set<int> sigIds;
        std::map<int, int> sigIdMap; // map of ids of DB signatures and mem signatures
        dbDriver->getAllNodeIds(sigIds, true);
        dbDriver->loadSignatures(std::list<int>(sigIds.begin(), sigIds.end()), signatures);
        dbDriver->loadNodeData(signatures);
        for (std::set<int>::iterator iter = sigIds.begin(); iter != sigIds.end(); ++iter)
        {
            sigIdMap.insert(std::pair<int, int>(*iter, nextMemSigId));
            nextMemSigId++;
        }
        _sigIdMaps.push_back(sigIdMap);

        // Read words from database
        UDEBUG("Read words from database...");
        std::set<int> wordIds;
        std::map<int, int> wordIdMap; // map of ids of DB words and mem words
        for (std::list<rtabmap::Signature *>::const_iterator iter = signatures.begin(); iter != signatures.end(); ++iter)
        {
            const std::multimap<int, cv::KeyPoint> &words = (*iter)->getWords();
            std::list<int> keys = uUniqueKeys(words);
            wordIds.insert(keys.begin(), keys.end());
        }
        std::list<rtabmap::VisualWord *> words;
        dbDriver->loadWords(wordIds, words);
        for (std::set<int>::const_iterator iter = wordIds.begin(); iter != wordIds.end(); ++iter)
        {
            wordIdMap.insert(std::pair<int, int>(*iter, nextMemWordId));
            nextMemWordId++;
        }
        _wordIdMaps.push_back(wordIdMap);

        // Add signatures to memory
        UDEBUG("Add signatures to memory...");
        std::map<int, rtabmap::Signature *> dbSignatureMap;
        for (std::list<rtabmap::Signature *>::iterator iter = signatures.begin(); iter != signatures.end(); ++iter)
        {
            rtabmap::Signature *dbSig = *iter;
            dbSignatureMap.insert(std::pair<int, rtabmap::Signature *>(dbSig->id(), dbSig));
            if (!dbSig->sensorData().imageCompressed().empty())
            {
                dbSig->sensorData().uncompressData();
            }
        }
        UDEBUG("Loading signatures done! (%d loaded)", int(dbSignatureMap.size()));
        _signatureMaps.push_back(dbSignatureMap); // temp push for optimzation, will pop later
        signatures.clear();

        // Optimize poses of signatures
        UDEBUG("Optimize poses of signatures...");
        optimizePoses(dbId);

        // Read labels from database before updating signature ids
        UDEBUG("Read labels from database...");
        std::list<Label *> labels = readLabels(dbId, dbUrl);
        allLabels.insert(allLabels.end(), labels.begin(), labels.end());

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

        UDEBUG("Closing database \"%s\"...", dbDriver->getUrl().c_str());
        dbDriver->closeConnection();
        dbDriver->join();
        delete dbDriver;
        dbDriver = NULL;
        UDEBUG("Closing database, done!");
    }
    _labels->addLabels(allLabels);

    return true;
}

void RTABMapDBAdapter::parseParameters(const rtabmap::ParametersMap &parameters)
{
    uInsert(parameters_, parameters);
}

void RTABMapDBAdapter::optimizePoses(int dbId)
{
    // TODO: why rbegin is guaranteed to have neighbors?
    int rootSigId = _signatureMaps.at(dbId).rbegin()->first;

    // Get all IDs linked to last signature
    std::map<int, int> ids = getNeighborsId(dbId, rootSigId, 0);

    UINFO("Optimize poses, ids.size() = %d", ids.size());

    // Get all metric constraints (the graph)
    std::map<int, rtabmap::Transform> poses;
    std::multimap<int, rtabmap::Link> links;
    getMetricConstraints(dbId, uKeysSet(ids), poses, links);

    // Optimize the graph
    rtabmap::Optimizer::Type optimizerType = rtabmap::Optimizer::kTypeTORO; // options: kTypeTORO, kTypeG2O, kTypeGTSAM, kTypeCVSBA
    rtabmap::Optimizer *graphOptimizer = rtabmap::Optimizer::create(optimizerType);
    std::map<int, rtabmap::Transform> optimizedPoseMap = graphOptimizer->optimize(poses.begin()->first, poses, links);
    delete graphOptimizer;

    // Update signatures poses in memory
    UDEBUG("Update signatures poses memory...");
    std::map<int, rtabmap::Signature *> signatureMap = _signatureMaps.at(dbId);
    for (std::map<int, rtabmap::Signature *>::const_iterator iter = signatureMap.begin(); iter != signatureMap.end(); ++iter)
    {
        rtabmap::Signature *sig = iter->second;
        const std::map<int, rtabmap::Transform>::const_iterator poseIter = optimizedPoseMap.find(sig->id());
        if (poseIter == optimizedPoseMap.end())
        {
            UFATAL("");
        }

        sig->setPose(poseIter->second);
    }
}

const std::map< int, std::list<Label *> > &RTABMapDBAdapter::getLabels() const
{
    return _labels->getLabels();
}

const Words *RTABMapDBAdapter::getWords() const
{
    return _words;
}

const std::map<int, Signature *> &RTABMapDBAdapter::getSignatures() const
{
    return _signatures->getSignatures();
}

std::vector<int> RTABMapDBAdapter::findKNearestSignatures(std::vector<int> wordIds, int k)
{
    return _signatures->findKNN(wordIds, k);
}

std::map<int, rtabmap::Link> RTABMapDBAdapter::getNeighborLinks(int dbId, int sigId) const
{
    std::map<int, rtabmap::Link> links;
    const rtabmap::Signature *sig = _signatureMaps.at(dbId).at(sigId);
    if (sig)
    {
        const std::map<int, rtabmap::Link> &allLinks = sig->getLinks();
        for (std::map<int, rtabmap::Link>::const_iterator iter = allLinks.begin(); iter != allLinks.end(); ++iter)
        {
            if (iter->second.type() == rtabmap::Link::kNeighbor ||
                    iter->second.type() == rtabmap::Link::kNeighborMerged)
            {
                links.insert(*iter);
            }
        }
    }
    else
    {
        UWARN("Cannot find signature %d in memory", sigId);
    }
    return links;
}

std::list<Label *> RTABMapDBAdapter::readLabels(int dbId, std::string dbUrl) const
{
    std::list<Label *> labels;
    sqlite3 *db = NULL;
    sqlite3_stmt *stmt = NULL;
    int rc;

    rc = sqlite3_open(dbUrl.c_str(), &db);
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
                UINFO("Read point (%lf,%lf,%lf) with label %s in database %s", pWorld.x, pWorld.y, pWorld.z, name.c_str(), dbUrl.c_str());
            }
        }
    }
    else
    {
        UWARN("Could not read database %s: %s", dbUrl.c_str(), sqlite3_errmsg(db));
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

// return map<Id,Margin>, including signatureId
std::map<int, int> RTABMapDBAdapter::getNeighborsId(
    int dbId,
    int signatureId,
    bool incrementMarginOnLoop, // default false
    bool ignoreLoopIds, // default false
    bool ignoreIntermediateNodes // default false
) const
{
    //UDEBUG("signatureId=%d, neighborsMargin=%d", signatureId, margin);
    std::map<int, int> ids;
    if (signatureId <= 0)
    {
        return ids;
    }
    int nbLoadedFromDb = 0;
    std::list<int> curentMarginList;
    std::set<int> currentMargin;
    std::set<int> nextMargin;
    nextMargin.insert(signatureId);
    int m = 0;
    std::set<int> ignoredIds;
    while (nextMargin.size())
    {
        // insert more recent first (priority to be loaded first from the database below if set)
        curentMarginList = std::list<int>(nextMargin.rbegin(), nextMargin.rend());
        nextMargin.clear();

        for (std::list<int>::iterator jter = curentMarginList.begin(); jter != curentMarginList.end(); ++jter)
        {
            if (ids.find(*jter) == ids.end())
            {
                //UDEBUG("Added %d with margin %d", *jter, m);
                // Look up in STM/WM if all ids are here, if not... load them from the database
                const rtabmap::Signature *s = _signatureMaps.at(dbId).at(*jter);
                std::map<int, rtabmap::Link> tmpLinks;
                const std::map<int, rtabmap::Link> *links = &tmpLinks;
                UASSERT(s != NULL);
                if (!ignoreIntermediateNodes || s->getWeight() != -1)
                {
                    ids.insert(std::pair<int, int>(*jter, m));
                }
                else
                {
                    ignoredIds.insert(*jter);
                }
                links = &s->getLinks();

                // links
                for (std::map<int, rtabmap::Link>::const_iterator iter = links->begin(); iter != links->end(); ++iter)
                {
                    if (!uContains(ids, iter->first) && ignoredIds.find(iter->first) == ignoredIds.end())
                    {
                        UASSERT(iter->second.type() != rtabmap::Link::kUndef);
                        if (iter->second.type() == rtabmap::Link::kNeighbor ||
                                iter->second.type() == rtabmap::Link::kNeighborMerged)
                        {
                            if (ignoreIntermediateNodes && s->getWeight() == -1)
                            {
                                // stay on the same margin
                                if (currentMargin.insert(iter->first).second)
                                {
                                    curentMarginList.push_back(iter->first);
                                }
                            }
                            else
                            {
                                nextMargin.insert(iter->first);
                            }
                        }
                        else if (!ignoreLoopIds)
                        {
                            if (incrementMarginOnLoop)
                            {
                                nextMargin.insert(iter->first);
                            }
                            else
                            {
                                if (currentMargin.insert(iter->first).second)
                                {
                                    curentMarginList.push_back(iter->first);
                                }
                            }
                        }
                    }
                }
            }
        }
        ++m;
    }
    return ids;
}

// return all non-null poses
// return unique links between nodes (for neighbors: old->new, for loops: parent->child)
void RTABMapDBAdapter::getMetricConstraints(
    int dbId,
    const std::set<int> &ids,
    std::map<int, rtabmap::Transform> &poses,
    std::multimap<int, rtabmap::Link> &links)
{
    UDEBUG("");
    for (std::set<int>::const_iterator iter = ids.begin(); iter != ids.end(); ++iter)
    {
        const rtabmap::Signature *s = _signatureMaps.at(dbId).at(*iter);
        rtabmap::Transform pose = s->getPose();
        if (!pose.isNull())
        {
            poses.insert(std::make_pair(*iter, pose));
        }
    }

    for (std::set<int>::const_iterator iter = ids.begin(); iter != ids.end(); ++iter)
    {
        if (uContains(poses, *iter))
        {
            const rtabmap::Signature *s = _signatureMaps.at(dbId).at(*iter);
            UASSERT(s != NULL);
            std::map<int, rtabmap::Link> tmpLinks = s->getLinks();
            for (std::map<int, rtabmap::Link>::iterator jter = tmpLinks.begin(); jter != tmpLinks.end(); ++jter)
            {
                if (jter->second.isValid() &&
                        uContains(poses, jter->first) &&
                        rtabmap::graph::findLink(links, *iter, jter->first) == links.end())
                {
                    if ((jter->second.type() == rtabmap::Link::kNeighbor ||
                            jter->second.type() == rtabmap::Link::kNeighborMerged))
                    {
                        rtabmap::Link link = jter->second;
                        const rtabmap::Signature *s = _signatureMaps.at(dbId).at(jter->first);
                        UASSERT(s != NULL);
                        while (s && s->getWeight() == -1)
                        {
                            // skip to next neighbor, well we assume that bad signatures
                            // are only linked by max 2 neighbor links.
                            std::map<int, rtabmap::Link> n = this->getNeighborLinks(dbId, s->id());
                            UASSERT(n.size() <= 2);
                            std::map<int, rtabmap::Link>::iterator uter = n.upper_bound(s->id());
                            if (uter != n.end())
                            {
                                const rtabmap::Signature *s2 = _signatureMaps.at(dbId).at(uter->first);
                                if (s2)
                                {
                                    link = link.merge(uter->second, uter->second.type());
                                    poses.erase(s->id());
                                    s = s2;
                                }
                            }
                            else
                            {
                                break;
                            }
                        }

                        links.insert(std::make_pair(*iter, link));
                    }
                    else
                    {
                        links.insert(std::make_pair(*iter, jter->second));
                    }
                }
            }
        }
    }
}
