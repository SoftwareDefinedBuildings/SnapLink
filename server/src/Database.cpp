#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UProcessInfo.h>
#include <rtabmap/utilite/UMath.h>

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/EpipolarGeometry.h>
#include <rtabmap/core/VisualWord.h>
#include <rtabmap/core/Features2d.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/util3d_features.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d_correspondences.h>
#include <rtabmap/core/util3d_registration.h>
#include <rtabmap/core/util3d_surface.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_motion_estimation.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/core/Optimizer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

#include "Database.h"

const int Database::kIdStart = 0;

Database::Database() :
    _badSignaturesIgnored(rtabmap::Parameters::defaultMemBadSignaturesIgnored()),
    _feature2D(NULL),
    _vwd(NULL),
    _dbDriver(NULL),
{
}

bool Database::init(const std::string &dbUrl, const rtabmap::ParametersMap &parameters)
{
    UDEBUG("");

    _feature2D = rtabmap::Feature2D::create(parameters);
    _vwd = new rtabmap::VWDictionary(parameters);
    this->parseParameters(parameters);

    if (dbUrl.empty()) // so this Memory will be empty
    {
        return true;
    }

    _dbDriver = rtabmap::DBDriver::create(parameters);

    _dbDriver->setTimestampUpdateEnabled(true); // make sure that timestamp update is enabled (may be disabled above)
    if (!_dbDriver->openConnection(dbUrl))
    {
        UDEBUG("Connecting to database %s, path is invalid!", dbUrl.c_str());
        return false;
    }
    UDEBUG("Connecting to database %s done!", dbUrl.c_str());

    // Load the last working memory...
    std::list<rtabmap::Signature *> dbSignatures;
    UDEBUG("Loading all nodes to WM...");
    std::set<int> ids;
    _dbDriver->getAllNodeIds(ids, true);
    _dbDriver->loadSignatures(std::list<int>(ids.begin(), ids.end()), dbSignatures);
    _dbDriver->loadNodeData(dbSignatures);

    for (std::list<rtabmap::Signature *>::reverse_iterator iter = dbSignatures.rbegin(); iter != dbSignatures.rend(); ++iter)
    {
        // ignore bad signatures
        if (!((*iter)->isBadSignature() && _badSignaturesIgnored))
        {
            _signatures.insert(std::pair<int, rtabmap::Signature *>((*iter)->id(), *iter));
            if (!(*iter)->sensorData().imageCompressed().empty())
            {
                (*iter)->sensorData().uncompressData();
            }
        }
        else
        {
            delete *iter;
        }
    }
    UDEBUG("Loading signatures done! (%d loaded)", int(_signatures.size()));

    // Now load the dictionary if we have a connection
    UDEBUG("Loading dictionary...");
    // load all referenced words in working memory
    std::set<int> wordIds;
    const std::map<int, rtabmap::Signature *> &signatures = this->getSignatures();
    for (std::map<int, rtabmap::Signature *>::const_iterator i = signatures.begin(); i != signatures.end(); ++i)
    {
        const std::multimap<int, cv::KeyPoint> &words = i->second->getWords();
        std::list<int> keys = uUniqueKeys(words);
        wordIds.insert(keys.begin(), keys.end());
    }
    if (wordIds.size())
    {
        std::list<rtabmap::VisualWord *> words;
        _dbDriver->loadWords(wordIds, words);
        for (std::list<rtabmap::VisualWord *>::iterator iter = words.begin(); iter != words.end(); ++iter)
        {
            _vwd->addWord(*iter);
        }
        // Get Last word id
        int id = 0;
        _dbDriver->getLastWordId(id);
        _vwd->setLastWordId(id);
    }
    UDEBUG("%d words loaded!", _vwd->getUnusedWordsSize());
    _vwd->update();

    UDEBUG("Adding word references...");
    // Enable loaded signatures
    for (std::map<int, rtabmap::Signature *>::const_iterator i = signatures.begin(); i != signatures.end(); ++i)
    {
        rtabmap::Signature *s = this->_getSignature(i->first);
        UASSERT(s != 0);

        const std::multimap<int, cv::KeyPoint> &words = s->getWords();
        if (words.size())
        {
            UDEBUG("node=%d, word references=%d", s->id(), words.size());
            for (std::multimap<int, cv::KeyPoint>::const_iterator iter = words.begin(); iter != words.end(); ++iter)
            {
                _vwd->addWordRef(iter->first, i->first);
            }
            s->setEnabled(true);
        }
    }
    UDEBUG("Adding word references, done! (%d)", _vwd->getTotalActiveReferences());

    if (_vwd->getUnusedWordsSize())
    {
        UWARN("_vwd->getUnusedWordsSize() must be empty... size=%d", _vwd->getUnusedWordsSize());
    }
    UDEBUG("Total word references added = %d", _vwd->getTotalActiveReferences());

    optimizeGraph();

    return true;
}

void Database::close()
{
    if (_dbDriver)
    {
        UDEBUG("Closing database \"%s\"...", _dbDriver->getUrl().c_str());
        _dbDriver->closeConnection();
        delete _dbDriver;
        _dbDriver = NULL;
        UDEBUG("Closing database, done!");
    }
    UDEBUG("Clearing memory...");
    this->clear();
    UDEBUG("Clearing memory, done!");
}

Database::~Database()
{
    this->close();

    if (_feature2D)
    {
        delete _feature2D;
    }
    if (_vwd)
    {
        delete _vwd;
    }
}

void Database::parseParameters(const rtabmap::ParametersMap &parameters)
{
    uInsert(parameters_, parameters);

    UDEBUG("");
    rtabmap::ParametersMap::const_iterator iter;

    rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kMemBadSignaturesIgnored(), _badSignaturesIgnored);

    if (_dbDriver)
    {
        _dbDriver->parseParameters(parameters);
    }

    // Keypoint stuff
    if (_vwd)
    {
        _vwd->parseParameters(parameters);
    }

    //Keypoint detector
    UASSERT(_feature2D != 0);
    rtabmap::Feature2D::Type detectorStrategy = rtabmap::Feature2D::kFeatureUndef;
    if ((iter = parameters.find(rtabmap::Parameters::kKpDetectorStrategy())) != parameters.end())
    {
        detectorStrategy = (rtabmap::Feature2D::Type)std::atoi((*iter).second.c_str());
    }
    if (detectorStrategy != rtabmap::Feature2D::kFeatureUndef)
    {
        UDEBUG("new detector strategy %d", int(detectorStrategy));
        if (_feature2D)
        {
            delete _feature2D;
            _feature2D = 0;
        }

        _feature2D = rtabmap::Feature2D::create(detectorStrategy, parameters_);
    }
    else if (_feature2D)
    {
        _feature2D->parseParameters(parameters);
    }
}

void Database::optimizeGraph()
{
    if (getLastWorkingSignature())
    {
        // Get all IDs linked to last signature (including those in Long-Term Memory)
        std::map<int, int> ids = getNeighborsId(getLastWorkingSignature()->id(), 0);

        UINFO("Optimize poses, ids.size() = %d", ids.size());

        // Get all metric constraints (the graph)
        std::map<int, rtabmap::Transform> poses;
        std::multimap<int, rtabmap::Link> links;
        getMetricConstraints(uKeysSet(ids), poses, links);

        // Optimize the graph
        rtabmap::Optimizer::Type optimizerType = rtabmap::Optimizer::kTypeTORO; // options: kTypeTORO, kTypeG2O, kTypeGTSAM, kTypeCVSBA
        rtabmap::Optimizer *graphOptimizer = rtabmap::Optimizer::create(optimizerType);
        _optimizedPoses = graphOptimizer->optimize(poses.begin()->first, poses, links);
        delete graphOptimizer;
    }
}

const rtabmap::Signature *Database::getSignature(int id) const
{
    return _getSignature(id);
}

const std::map<int, rtabmap::Signature *> &Database::getSignatures() const
{
    return _signatures;
}

rtabmap::Signature *Database::_getSignature(int id) const
{
    return uValue(_signatures, id, (rtabmap::Signature *)0);
}

std::map<int, rtabmap::Link> Database::getNeighborLinks(int signatureId) const
{
    std::map<int, rtabmap::Link> links;
    rtabmap::Signature *s = uValue(_signatures, signatureId, (rtabmap::Signature *)0);
    if (s)
    {
        const std::map<int, rtabmap::Link> &allLinks = s->getLinks();
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
        UWARN("Cannot find signature %d in memory", signatureId);
    }
    return links;
}

// return map<Id,Margin>, including signatureId
std::map<int, int> Database::getNeighborsId(
    int signatureId,
    int maxGraphDepth, // 0 means infinite margin
    bool incrementMarginOnLoop, // default false
    bool ignoreLoopIds, // default false
    bool ignoreIntermediateNodes // default false
) const
{
    UASSERT(maxGraphDepth >= 0);
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
    while ((maxGraphDepth == 0 || m < maxGraphDepth) && nextMargin.size())
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
                const rtabmap::Signature *s = this->getSignature(*jter);
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

void Database::clear()
{
    UDEBUG("");

    this->cleanUnusedWords();

    if (_dbDriver)
    {
        _dbDriver->emptyTrashes();
        _dbDriver->join();
    }
    if (_dbDriver)
    {
        // make sure time_enter in database is at least 1 second
        // after for the next stuf added to database
        uSleep(1500);
    }

    //Get the tree root (parents)
    std::map<int, rtabmap::Signature *> mem = _signatures;
    for (std::map<int, rtabmap::Signature *>::iterator i = mem.begin(); i != mem.end(); ++i)
    {
        if (i->second)
        {
            UDEBUG("deleting from the memory: %d", i->first);
            this->moveToTrash(i->second);
        }
    }

    if (_signatures.size() != 0)
    {
        ULOGGER_ERROR("_signatures must be empty here, size=%d", _signatures.size());
    }
    _signatures.clear();

    UDEBUG("");
    // Wait until the db trash has finished cleaning the memory
    if (_dbDriver)
    {
        _dbDriver->emptyTrashes();
    }
    UDEBUG("");

    if (_dbDriver)
    {
        _dbDriver->join(true);
        cleanUnusedWords();
        _dbDriver->emptyTrashes();
    }
    else
    {
        cleanUnusedWords();
    }
    if (_vwd)
    {
        _vwd->clear();
    }
    UDEBUG("");
}

void Database::emptyTrash()
{
    if (_dbDriver)
    {
        _dbDriver->emptyTrashes(true);
    }
}

/**
 * If saveToDatabase=false, deleted words are filled in deletedWords.
 */
void Database::moveToTrash(rtabmap::Signature *s, bool keepLinkedToGraph)
{
    UDEBUG("id=%d", s ? s->id() : 0);
    if (s)
    {
        this->disableWordsRef(s->id());
        if (!keepLinkedToGraph)
        {
            std::list<int> keys = uUniqueKeys(s->getWords());
            for (std::list<int>::const_iterator i = keys.begin(); i != keys.end(); ++i)
            {
                // assume just removed word doesn't have any other references
                rtabmap::VisualWord *w = _vwd->getUnusedWord(*i);
                if (w)
                {
                    std::vector<rtabmap::VisualWord *> wordToDelete;
                    wordToDelete.push_back(w);
                    _vwd->removeWords(wordToDelete);
                    delete w;
                }
            }
        }

        _signatures.erase(s->id());

        delete s;
    }
}

const rtabmap::Signature *Database::getLastWorkingSignature() const
{
    UDEBUG("");
    return this->_getSignature(_signatures.rbegin()->first);
}

rtabmap::Transform Database::getOptimizedPose(int signatureId) const
{
    rtabmap::Transform pose;
    const std::map<int, rtabmap::Transform>::const_iterator poseIter = _optimizedPoses.find(signatureId);
    if (poseIter != _optimizedPoses.end())
    {
        pose = poseIter->second;
    }

    return pose;
}

void Database::disableWordsRef(int signatureId)
{
    UDEBUG("id=%d", signatureId);

    rtabmap::Signature *ss = this->_getSignature(signatureId);
    if (ss && ss->isEnabled())
    {
        const std::multimap<int, cv::KeyPoint> &words = ss->getWords();
        const std::list<int> &keys = uUniqueKeys(words);
        int count = _vwd->getTotalActiveReferences();
        // First remove all references
        for (std::list<int>::const_iterator i = keys.begin(); i != keys.end(); ++i)
        {
            _vwd->removeAllWordRef(*i, signatureId);
        }

        count -= _vwd->getTotalActiveReferences();
        ss->setEnabled(false);
        UDEBUG("%d words total ref removed from signature %d... (total active ref = %d)", count, ss->id(), _vwd->getTotalActiveReferences());
    }
}

void Database::cleanUnusedWords()
{
    if (_vwd->isIncremental())
    {
        std::vector<rtabmap::VisualWord *> removedWords = _vwd->getUnusedWords();
        UDEBUG("Removing %d words (dictionary size=%d)...", removedWords.size(), _vwd->getVisualWords().size());
        if (removedWords.size())
        {
            // remove them from the dictionary
            _vwd->removeWords(removedWords);

            for (unsigned int i = 0; i < removedWords.size(); ++i)
            {
                delete removedWords[i];
            }
        }
    }
}

// return all non-null poses
// return unique links between nodes (for neighbors: old->new, for loops: parent->child)
void Database::getMetricConstraints(
    const std::set<int> &ids,
    std::map<int, rtabmap::Transform> &poses,
    std::multimap<int, rtabmap::Link> &links)
{
    UDEBUG("");
    for (std::set<int>::const_iterator iter = ids.begin(); iter != ids.end(); ++iter)
    {
        const rtabmap::Signature *s = getSignature(*iter);
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
            const rtabmap::Signature *s = getSignature(*iter);
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
                        const rtabmap::Signature *s = this->getSignature(jter->first);
                        UASSERT(s != NULL);
                        while (s && s->getWeight() == -1)
                        {
                            // skip to next neighbor, well we assume that bad signatures
                            // are only linked by max 2 neighbor links.
                            std::map<int, rtabmap::Link> n = this->getNeighborLinks(s->id());
                            UASSERT(n.size() <= 2);
                            std::map<int, rtabmap::Link>::iterator uter = n.upper_bound(s->id());
                            if (uter != n.end())
                            {
                                const rtabmap::Signature *s2 = this->getSignature(uter->first);
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
