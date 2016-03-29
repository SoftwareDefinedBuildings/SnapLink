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

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

#include "MemoryLoc.h"

const int MemoryLoc::kIdStart = 0;

MemoryLoc::MemoryLoc() :
    _incrementalMemory(rtabmap::Parameters::defaultMemIncrementalMemory()),
    _badSignaturesIgnored(rtabmap::Parameters::defaultMemBadSignaturesIgnored()),
    _idCount(kIdStart),
    _lastSignature(0),
    _feature2D(NULL),
    _vwd(NULL),
    _dbDriver(NULL),

    _minInliers(rtabmap::Parameters::defaultVisMinInliers()),
    _iterations(rtabmap::Parameters::defaultVisIterations()),
    _pnpRefineIterations(rtabmap::Parameters::defaultVisPnPRefineIterations()),
    _pnpReprojError(rtabmap::Parameters::defaultVisPnPReprojError()),
    _pnpFlags(rtabmap::Parameters::defaultVisPnPFlags())
{
}

bool MemoryLoc::init(const std::string &dbUrl, const rtabmap::ParametersMap &parameters)
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

    for (std::list<rtabmap::Signature *>::reverse_iterator iter = dbSignatures.rbegin(); iter != dbSignatures.rend(); ++iter)
    {
        // ignore bad signatures
        if (!((*iter)->isBadSignature() && _badSignaturesIgnored))
        {
            _signatures.insert(std::pair<int, rtabmap::Signature *>((*iter)->id(), *iter));
        }
        else
        {
            delete *iter;
        }
    }
    UDEBUG("Loading signatures done! (%d loaded)", int(_signatures.size()));

    // Assign the last signature
    if (_signatures.size() > 0)
    {
        _lastSignature = uValue(_signatures, _signatures.rbegin()->first, (rtabmap::Signature *)0);
    }

    // Last id
    _dbDriver->getLastNodeId(_idCount);

    UDEBUG("ids start with %d", _idCount + 1);

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

    return true;
}

void MemoryLoc::close()
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

MemoryLoc::~MemoryLoc()
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

void MemoryLoc::parseParameters(const rtabmap::ParametersMap &parameters)
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

    rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kVisMinInliers(), _minInliers);
    rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kVisIterations(), _iterations);
    rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kVisPnPRefineIterations(), _pnpRefineIterations);
    rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kVisPnPReprojError(), _pnpReprojError);
    rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kVisPnPFlags(), _pnpFlags);
}

void MemoryLoc::preUpdate()
{
    this->cleanUnusedWords();
    if (_vwd)
    {
        _vwd->update();
    }
}

bool MemoryLoc::update(const rtabmap::SensorData &data)
{
    UDEBUG("");
    UTimer timer;
    UTimer totalTimer;
    timer.start();
    float t;

    //============================================================
    // Pre update...
    //============================================================
    UDEBUG("pre-updating...");
    this->preUpdate();
    t = timer.ticks() * 1000;
    UDEBUG("time preUpdate=%f ms", t);

    //============================================================
    // Create a signature with the image received.
    //============================================================
    rtabmap::Signature *signature = this->createSignature(data);
    if (signature == 0)
    {
        UERROR("Failed to create a signature...");
        return false;
    }

    t = timer.ticks() * 1000;
    UDEBUG("time creating signature=%f ms", t);

    // It will be added to the short-term memory, no need to delete it...
    this->addSignature(signature);

    _lastSignature = signature;

    UDEBUG("totalTimer = %fs", totalTimer.ticks());

    return true;
}

void MemoryLoc::addSignature(rtabmap::Signature *signature)
{
    if (signature)
    {
        UDEBUG("adding %d", signature->id());
        _signatures.insert(_signatures.end(), std::pair<int, rtabmap::Signature *>(signature->id(), signature));

        if (_vwd)
        {
            UDEBUG("%d words ref for the signature %d", signature->getWords().size(), signature->id());
        }
        if (signature->getWords().size())
        {
            signature->setEnabled(true);
        }
    }
}

const rtabmap::Signature *MemoryLoc::getSignature(int id) const
{
    return _getSignature(id);
}

rtabmap::Signature *MemoryLoc::_getSignature(int id) const
{
    return uValue(_signatures, id, (rtabmap::Signature *)0);
}

std::map<int, rtabmap::Link> MemoryLoc::getNeighborLinks(
    int signatureId,
    bool lookInDatabase) const
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
    else if (lookInDatabase && _dbDriver)
    {
        std::map<int, rtabmap::Link> neighbors;
        _dbDriver->loadLinks(signatureId, neighbors);
        for (std::map<int, rtabmap::Link>::iterator iter = neighbors.begin(); iter != neighbors.end();)
        {
            if (iter->second.type() != rtabmap::Link::kNeighbor &&
                    iter->second.type() != rtabmap::Link::kNeighborMerged)
            {
                neighbors.erase(iter++);
            }
            else
            {
                ++iter;
            }
        }
    }
    else
    {
        UWARN("Cannot find signature %d in memory", signatureId);
    }
    return links;
}

std::map<int, rtabmap::Link> MemoryLoc::getLinks(
    int signatureId,
    bool lookInDatabase) const
{
    std::map<int, rtabmap::Link> links;
    rtabmap::Signature *s = uValue(_signatures, signatureId, (rtabmap::Signature *)0);
    if (s)
    {
        links = s->getLinks();
    }
    else if (lookInDatabase && _dbDriver)
    {
        _dbDriver->loadLinks(signatureId, links, rtabmap::Link::kUndef);
    }
    else
    {
        UWARN("Cannot find signature %d in memory", signatureId);
    }
    return links;
}

// return map<Id,Margin>, including signatureId
// maxCheckedInDatabase = -1 means no limit to check in database (default)
// maxCheckedInDatabase = 0 means don't check in database
std::map<int, int> MemoryLoc::getNeighborsId(
    int signatureId,
    int maxGraphDepth, // 0 means infinite margin
    int maxCheckedInDatabase, // default -1 (no limit)
    bool incrementMarginOnLoop, // default false
    bool ignoreLoopIds, // default false
    bool ignoreIntermediateNodes, // default false
    double *dbAccessTime
) const
{
    UASSERT(maxGraphDepth >= 0);
    //UDEBUG("signatureId=%d, neighborsMargin=%d", signatureId, margin);
    if (dbAccessTime)
    {
        *dbAccessTime = 0;
    }
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
                if (s)
                {
                    if (!ignoreIntermediateNodes || s->getWeight() != -1)
                    {
                        ids.insert(std::pair<int, int>(*jter, m));
                    }
                    else
                    {
                        ignoredIds.insert(*jter);
                    }

                    links = &s->getLinks();
                }
                else if (maxCheckedInDatabase == -1 || (maxCheckedInDatabase > 0 && _dbDriver && nbLoadedFromDb < maxCheckedInDatabase))
                {
                    ++nbLoadedFromDb;
                    ids.insert(std::pair<int, int>(*jter, m));

                    UTimer timer;
                    _dbDriver->loadLinks(*jter, tmpLinks);
                    if (dbAccessTime)
                    {
                        *dbAccessTime += timer.getElapsedTime();
                    }
                }

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

int MemoryLoc::getNextId()
{
    return ++_idCount;
}

void MemoryLoc::clear()
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
            UDEBUG("deleting from the working and the short-term memory: %d", i->first);
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
    _lastSignature = 0;
    _idCount = kIdStart;

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

/**
 * Compute the likelihood of the signature with some others in the memory.
 * Important: Assuming that all other ids are under 'signature' id.
 * If an error occurs, the result is empty.
 */
std::map<int, float> MemoryLoc::computeLikelihood(const rtabmap::Signature *signature, const std::list<int> &ids)
{
    UTimer timer;
    timer.start();
    std::map<int, float> likelihood;

    if (!signature)
    {
        ULOGGER_ERROR("The signature is null");
        return likelihood;
    }
    else if (ids.empty())
    {
        UWARN("ids list is empty");
        return likelihood;
    }

    for (std::list<int>::const_iterator iter = ids.begin(); iter != ids.end(); ++iter)
    {
        float sim = 0.0f;
        if (*iter > 0)
        {
            const rtabmap::Signature *sB = this->getSignature(*iter);
            if (!sB)
            {
                UFATAL("Signature %d not found in WM ?!?", *iter);
            }
            sim = signature->compareTo(*sB);
        }

        likelihood.insert(likelihood.end(), std::pair<int, float>(*iter, sim));
    }

    UDEBUG("compute likelihood (similarity)... %f s", timer.ticks());
    return likelihood;
}

rtabmap::Transform MemoryLoc::computeGlobalVisualTransform(const std::vector<int> &oldIds, int newId) const
{
    std::vector<const rtabmap::Signature *> oldSigs;
    for (std::vector<int>::const_iterator it = oldIds.begin() ; it != oldIds.end(); it++)
    {
        const rtabmap::Signature *oldSig = getSignature(*it);
        if (oldSig == NULL)
        {
            return rtabmap::Transform();
        }
        oldSigs.push_back(oldSig);
    }

    const rtabmap::Signature *newSig = getSignature(newId);
    if (newSig == NULL)
    {
        return rtabmap::Transform();
    }

    return computeGlobalVisualTransform(oldSigs, newSig);
}

rtabmap::Transform MemoryLoc::computeGlobalVisualTransform(const std::vector<const rtabmap::Signature *> &oldSigs, const rtabmap::Signature *newSig) const
{
    if (oldSigs.size() == 0 || newSig == NULL)
    {
        return rtabmap::Transform();
    }

    rtabmap::Transform transform;
    std::string msg;

    int inliersCount = 0;
    double variance = 1.0;

    std::multimap<int, cv::Point3f> words3;

    const std::vector<const rtabmap::Signature *>::const_iterator firstSig = oldSigs.begin();
    const rtabmap::Transform &guessPose = (*firstSig)->getPose();

    for (std::vector<const rtabmap::Signature *>::const_iterator sigIter = oldSigs.begin(); sigIter != oldSigs.end(); sigIter++)
    {
        rtabmap::Transform pose = (*sigIter)->getPose();
        const std::multimap<int, cv::Point3f> &sigWords3 = (*sigIter)->getWords3();
        std::multimap<int, cv::Point3f>::const_iterator word3Iter;
        for (word3Iter = sigWords3.begin(); word3Iter != sigWords3.end(); word3Iter++)
        {
            cv::Point3f point3 = rtabmap::util3d::transformPoint(word3Iter->second, pose);
            words3.insert(std::pair<int, cv::Point3f>(word3Iter->first, point3));
        }
    }

    // 3D to 2D (PnP)
    if ((int)words3.size() >= _minInliers && (int)newSig->getWords().size() >= _minInliers)
    {
        const rtabmap::CameraModel &cameraModel = newSig->sensorData().cameraModels()[0];

        std::vector<int> matches;
        std::vector<int> inliers;
        transform = rtabmap::util3d::estimateMotion3DTo2D(
                        uMultimapToMapUnique(words3),
                        uMultimapToMapUnique(newSig->getWords()),
                        cameraModel, // TODO: cameraModel.localTransform has to be the same for all images
                        _minInliers,
                        _iterations,
                        _pnpReprojError,
                        _pnpFlags,
                        _pnpRefineIterations,
                        guessPose, // use the first signature's pose as a guess
                        uMultimapToMapUnique(newSig->getWords3()),
                        &variance,
                        &matches,
                        &inliers);
        inliersCount = (int)inliers.size();
        if (transform.isNull())
        {
            msg = uFormat("Not enough inliers %d/%d between the old signatures and %d", inliersCount, _minInliers, newSig->id());
            UINFO(msg.c_str());
        }
    }
    else
    {
        msg = uFormat("Not enough features in images (old=%d, new=%d, min=%d)", (int)words3.size(), (int)newSig->getWords().size(), _minInliers);
        UINFO(msg.c_str());
    }

    // TODO check RegistrationVis.cpp to see whether this is necessary
    if (!transform.isNull())
    {
        // verify if it is a 180 degree transform, well verify > 90
        float x, y, z, roll, pitch, yaw;
        transform.inverse().getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
        if (fabs(roll) > CV_PI / 2 || fabs(pitch) > CV_PI / 2 || fabs(yaw) > CV_PI / 2)
        {
            transform.setNull();
            msg = uFormat("Too large rotation detected! (roll=%f, pitch=%f, yaw=%f)", roll, pitch, yaw);
            UWARN(msg.c_str());
        }
    }

    UDEBUG("transform=%s", transform.prettyPrint().c_str());
    return transform;
}

void MemoryLoc::emptyTrash()
{
    if (_dbDriver)
    {
        _dbDriver->emptyTrashes(true);
    }
}

class WeightAgeIdKey
{
public:
    WeightAgeIdKey(int w, double a, int i) :
        weight(w),
        age(a),
        id(i) {}
    bool operator<(const WeightAgeIdKey &k) const
    {
        if (weight < k.weight)
        {
            return true;
        }
        else if (weight == k.weight)
        {
            if (age < k.age)
            {
                return true;
            }
            else if (age == k.age)
            {
                if (id < k.id)
                {
                    return true;
                }
            }
        }
        return false;
    }
    int weight, age, id;
};

/**
 * If saveToDatabase=false, deleted words are filled in deletedWords.
 */
void MemoryLoc::moveToTrash(rtabmap::Signature *s, bool keepLinkedToGraph)
{
    UDEBUG("id=%d", s ? s->id() : 0);
    if (s)
    {
        // If not saved to database or it is a bad signature (not saved), remove links!
        if (!keepLinkedToGraph || (!s->isSaved() && s->isBadSignature() && _badSignaturesIgnored))
        {
            UDEBUG("remove links");
            const std::map<int, rtabmap::Link> &links = s->getLinks();
            for (std::map<int, rtabmap::Link>::const_iterator iter = links.begin(); iter != links.end(); ++iter)
            {
                UDEBUG("remove link");
                rtabmap::Signature *sTo = this->_getSignature(iter->first);
                // neighbor to s
                UASSERT_MSG(sTo != 0,
                            uFormat("A neighbor (%d) of the deleted location %d is "
                                    "not found in WM/STM! Are you deleting a location "
                                    "outside the STM?", iter->first, s->id()).c_str());

                if (iter->first > s->id() && links.size() > 1 && sTo->hasLink(s->id()))
                {
                    UWARN("Link %d of %d is newer, removing neighbor link "
                          "may split the map!",
                          iter->first, s->id());
                }

                // child
                if (iter->second.type() == rtabmap::Link::kGlobalClosure && s->id() > sTo->id())
                {
                    sTo->setWeight(sTo->getWeight() + s->getWeight()); // copy weight
                }

                sTo->removeLink(s->id());

            }
            s->removeLinks(); // remove all links
            s->setWeight(0);
            s->setLabel(""); // reset label
        }
        else
        {
            UDEBUG("remove virtual links");
            // Make sure that virtual links are removed.
            // It should be called before the signature is
            // removed from _signatures below.
            removeVirtualLinks(s->id());
        }

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

        if (_lastSignature == s)
        {
            _lastSignature = 0;
            if (_signatures.size())
            {
                _lastSignature = this->_getSignature(_signatures.rbegin()->first);
            }
        }

        delete s;
    }
}

const rtabmap::Signature *MemoryLoc::getLastWorkingSignature() const
{
    UDEBUG("");
    return _lastSignature;
}

void MemoryLoc::deleteLocation(int locationId)
{
    UDEBUG("Deleting location %d", locationId);
    rtabmap::Signature *location = _getSignature(locationId);
    if (location)
    {
        this->moveToTrash(location, false);
    }
}

void MemoryLoc::removeVirtualLinks(int signatureId)
{
    UDEBUG("");
    rtabmap::Signature *s = this->_getSignature(signatureId);
    if (s)
    {
        const std::map<int, rtabmap::Link> &links = s->getLinks();
        for (std::map<int, rtabmap::Link>::const_iterator iter = links.begin(); iter != links.end(); ++iter)
        {
            if (iter->second.type() == rtabmap::Link::kVirtualClosure)
            {
                rtabmap::Signature *sTo = this->_getSignature(iter->first);
                if (sTo)
                {
                    sTo->removeLink(s->id());
                }
                else
                {
                    UERROR("Link %d of %d not in WM/STM?!?", iter->first, s->id());
                }
            }
        }
        s->removeVirtualLinks();
    }
    else
    {
        UERROR("Signature %d not in WM/STM?!?", signatureId);
    }
}

rtabmap::Transform MemoryLoc::getOdomPose(int signatureId, bool lookInDatabase) const
{
    rtabmap::Transform pose, groundTruth;
    int mapId, weight;
    std::string label;
    double stamp;
    getNodeInfo(signatureId, pose, mapId, weight, label, stamp, groundTruth, lookInDatabase);
    return pose;
}

rtabmap::Transform MemoryLoc::getGroundTruthPose(int signatureId, bool lookInDatabase) const
{
    rtabmap::Transform pose, groundTruth;
    int mapId, weight;
    std::string label;
    double stamp;
    getNodeInfo(signatureId, pose, mapId, weight, label, stamp, groundTruth, lookInDatabase);
    return groundTruth;
}

bool MemoryLoc::getNodeInfo(int signatureId,
                            rtabmap::Transform &odomPose,
                            int &mapId,
                            int &weight,
                            std::string &label,
                            double &stamp,
                            rtabmap::Transform &groundTruth,
                            bool lookInDatabase) const
{
    const rtabmap::Signature *s = this->getSignature(signatureId);
    if (s)
    {
        odomPose = s->getPose();
        mapId = s->mapId();
        weight = s->getWeight();
        label = s->getLabel();
        stamp = s->getStamp();
        groundTruth = s->getGroundTruthPose();
        return true;
    }
    else if (lookInDatabase && _dbDriver)
    {
        return _dbDriver->getNodeInfo(signatureId, odomPose, mapId, weight, label, stamp, groundTruth);
    }
    return false;
}

rtabmap::Signature *MemoryLoc::createSignature(const rtabmap::SensorData &data)
{
    UDEBUG("");
    UASSERT(data.imageRaw().empty() ||
            data.imageRaw().type() == CV_8UC1 ||
            data.imageRaw().type() == CV_8UC3);

    UTimer timer;
    timer.start();
    float t;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    int id = this->getNextId();

    if (_feature2D->getMaxFeatures() >= 0 && !data.imageRaw().empty())
    {
        UINFO("Extract features");
        cv::Mat imageMono;
        if (data.imageRaw().channels() == 3)
        {
            cv::cvtColor(data.imageRaw(), imageMono, CV_BGR2GRAY);
        }
        else
        {
            imageMono = data.imageRaw();
        }

        keypoints = _feature2D->generateKeypoints(imageMono);
        t = timer.ticks();
        UDEBUG("time keypoints (%d) = %fs", (int)keypoints.size(), t);

        descriptors = _feature2D->generateDescriptors(imageMono, keypoints);
        t = timer.ticks();
        UDEBUG("time descriptors (%d) = %fs", descriptors.rows, t);
    }
    else if (data.imageRaw().empty())
    {
        UDEBUG("Empty image, cannot extract features...");
    }
    else if (_feature2D->getMaxFeatures() < 0)
    {
        UDEBUG("_feature2D->getMaxFeatures()(%d<0) so don't extract any features...", _feature2D->getMaxFeatures());
    }

    std::list<int> wordIds;
    if (descriptors.rows)
    {
        t = timer.ticks();
        UDEBUG("time descriptor (%d of size=%d) = %fs", descriptors.rows, descriptors.cols, t);

        wordIds = _vwd->addNewWords(descriptors, id);
        t = timer.ticks();
        UDEBUG("time addNewWords %fs", t);
    }
    else if (id > 0)
    {
        UDEBUG("id %d is a bad signature", id);
    }

    std::multimap<int, cv::KeyPoint> words;
    if (wordIds.size() > 0)
    {
        UASSERT(wordIds.size() == keypoints.size());
        unsigned int i = 0;
        for (std::list<int>::iterator iter = wordIds.begin(); iter != wordIds.end() && i < keypoints.size(); ++iter, ++i)
        {
            words.insert(std::pair<int, cv::KeyPoint>(*iter, keypoints[i]));
        }
    }

    cv::Mat image = data.imageRaw();
    std::vector<rtabmap::CameraModel> cameraModels = data.cameraModels();

    UDEBUG("bin data not kept");
    rtabmap::Signature *s = new rtabmap::Signature(id);

    s->sensorData().setId(id);
    s->sensorData().setCameraModels(cameraModels);

    s->setWords(words);

    // set raw data
    s->sensorData().setImageRaw(image);

    t = timer.ticks();
    UDEBUG("time compressing data (id=%d) %fs", id, t);
    if (words.size())
    {
        s->setEnabled(true); // All references are already activated in the dictionary at this point (see _vwd->addNewWords())
    }
    return s;
}

void MemoryLoc::disableWordsRef(int signatureId)
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

void MemoryLoc::cleanUnusedWords()
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
                if (_dbDriver)
                {
                    _dbDriver->asyncSave(removedWords[i]);
                }
                else
                {
                    delete removedWords[i];
                }
            }
        }
    }
}

// return all non-null poses
// return unique links between nodes (for neighbors: old->new, for loops: parent->child)
void MemoryLoc::getMetricConstraints(
    const std::set<int> &ids,
    std::map<int, rtabmap::Transform> &poses,
    std::multimap<int, rtabmap::Link> &links,
    bool lookInDatabase)
{
    UDEBUG("");
    for (std::set<int>::const_iterator iter = ids.begin(); iter != ids.end(); ++iter)
    {
        rtabmap::Transform pose = getOdomPose(*iter, lookInDatabase);
        if (!pose.isNull())
        {
            poses.insert(std::make_pair(*iter, pose));
        }
    }

    for (std::set<int>::const_iterator iter = ids.begin(); iter != ids.end(); ++iter)
    {
        if (uContains(poses, *iter))
        {
            std::map<int, rtabmap::Link> tmpLinks = getLinks(*iter, lookInDatabase);
            for (std::map<int, rtabmap::Link>::iterator jter = tmpLinks.begin(); jter != tmpLinks.end(); ++jter)
            {
                if (jter->second.isValid() &&
                        uContains(poses, jter->first) &&
                        rtabmap::graph::findLink(links, *iter, jter->first) == links.end())
                {
                    if (!lookInDatabase &&
                            (jter->second.type() == rtabmap::Link::kNeighbor ||
                             jter->second.type() == rtabmap::Link::kNeighborMerged))
                    {
                        rtabmap::Link link = jter->second;
                        const rtabmap::Signature *s = this->getSignature(jter->first);
                        UASSERT(s != 0);
                        while (s && s->getWeight() == -1)
                        {
                            // skip to next neighbor, well we assume that bad signatures
                            // are only linked by max 2 neighbor links.
                            std::map<int, rtabmap::Link> n = this->getNeighborLinks(s->id(), false);
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
