#include <rtabmap/core/VisualWord.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/UtiLite.h>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <rtflann/flann.hpp>
#include <fstream>
#include <string>
#include "VWDictFixed.h"

class FlannIndex
{
public:
    FlannIndex():
        index_(0),
        nextIndex_(0),
        featuresType_(0),
        featuresDim_(0),
        isLSH_(false),
        useDistanceL1_(false)
    {
    }
    virtual ~FlannIndex()
    {
        this->release();
    }

    void release()
    {
        if (index_)
        {
            if (featuresType_ == CV_8UC1)
            {
                delete(rtflann::Index<rtflann::Hamming<unsigned char> > *)index_;
            }
            else
            {
                if (useDistanceL1_)
                {
                    delete(rtflann::Index<rtflann::L1<float> > *)index_;
                }
                else
                {
                    delete(rtflann::Index<rtflann::L2<float> > *)index_;
                }
            }
            index_ = 0;
        }
        nextIndex_ = 0;
        isLSH_ = false;
        addedDescriptors_.clear();
        removedIndexes_.clear();
    }

    unsigned int indexedFeatures() const
    {
        if (!index_)
        {
            return 0;
        }
        if (featuresType_ == CV_8UC1)
        {
            return ((const rtflann::Index<rtflann::Hamming<unsigned char> > *)index_)->size();
        }
        else
        {
            if (useDistanceL1_)
            {
                return ((const rtflann::Index<rtflann::L1<float> > *)index_)->size();
            }
            else
            {
                return ((const rtflann::Index<rtflann::L2<float> > *)index_)->size();
            }
        }
    }

    // return KB
    unsigned int memoryUsed() const
    {
        if (!index_)
        {
            return 0;
        }
        if (featuresType_ == CV_8UC1)
        {
            return ((const rtflann::Index<rtflann::Hamming<unsigned char> > *)index_)->usedMemory() / 1000;
        }
        else
        {
            if (useDistanceL1_)
            {
                return ((const rtflann::Index<rtflann::L1<float> > *)index_)->usedMemory() / 1000;
            }
            else
            {
                return ((const rtflann::Index<rtflann::L2<float> > *)index_)->usedMemory() / 1000;
            }
        }
    }

    // Note that useDistanceL1 doesn't have any effect if LSH is used
    void build(
        const cv::Mat &features,
        const rtflann::IndexParams &params,
        bool useDistanceL1)
    {
        this->release();
        UASSERT(index_ == 0);
        UASSERT(features.type() == CV_32FC1 || features.type() == CV_8UC1);
        featuresType_ = features.type();
        featuresDim_ = features.cols;
        useDistanceL1_ = useDistanceL1;

        if (featuresType_ == CV_8UC1)
        {
            rtflann::Matrix<unsigned char> dataset(features.data, features.rows, features.cols);
            index_ = new rtflann::Index<rtflann::Hamming<unsigned char> >(dataset, params);
            ((rtflann::Index<rtflann::Hamming<unsigned char> > *)index_)->buildIndex();
        }
        else
        {
            rtflann::Matrix<float> dataset((float *)features.data, features.rows, features.cols);
            if (useDistanceL1_)
            {
                index_ = new rtflann::Index<rtflann::L1<float> >(dataset, params);
                ((rtflann::Index<rtflann::L1<float> > *)index_)->buildIndex();
            }
            else
            {
                index_ = new rtflann::Index<rtflann::L2<float> >(dataset, params);
                ((rtflann::Index<rtflann::L2<float> > *)index_)->buildIndex();
            }
        }

        if (features.rows == 1)
        {
            // incremental FLANN
            addedDescriptors_.insert(std::make_pair(nextIndex_, features));
        }
        // else assume that the features are kept in memory outside this class (e.g., dataTree_)

        nextIndex_ = features.rows;
    }

    bool isBuilt()
    {
        return index_ != 0;
    }

    int featuresType() const
    {
        return featuresType_;
    }
    int featuresDim() const
    {
        return featuresDim_;
    }

    unsigned int addPoint(const cv::Mat &feature)
    {
        if (!index_)
        {
            UERROR("Flann index not yet created!");
            return 0;
        }
        UASSERT(feature.type() == featuresType_);
        UASSERT(feature.cols == featuresDim_);
        UASSERT(feature.rows == 1);
        if (featuresType_ == CV_8UC1)
        {
            rtflann::Matrix<unsigned char> point(feature.data, feature.rows, feature.cols);
            rtflann::Index<rtflann::Hamming<unsigned char> > *index = (rtflann::Index<rtflann::Hamming<unsigned char> > *)index_;
            index->addPoints(point, 0);
            // Rebuild index if it doubles in size
            if (index->sizeAtBuild() * 2 < index->size() + index->removedCount())
            {
                // clean not used features
                for (std::list<int>::iterator iter = removedIndexes_.begin(); iter != removedIndexes_.end(); ++iter)
                {
                    addedDescriptors_.erase(*iter);
                }
                removedIndexes_.clear();
                index->buildIndex();
            }
        }
        else
        {
            rtflann::Matrix<float> point((float *)feature.data, feature.rows, feature.cols);
            if (useDistanceL1_)
            {
                rtflann::Index<rtflann::L1<float> > *index = (rtflann::Index<rtflann::L1<float> > *)index_;
                index->addPoints(point, 0);
                // Rebuild index if it doubles in size
                if (index->sizeAtBuild() * 2 < index->size() + index->removedCount())
                {
                    // clean not used features
                    for (std::list<int>::iterator iter = removedIndexes_.begin(); iter != removedIndexes_.end(); ++iter)
                    {
                        addedDescriptors_.erase(*iter);
                    }
                    removedIndexes_.clear();
                    index->buildIndex();
                }
            }
            else
            {
                rtflann::Index<rtflann::L2<float> > *index = (rtflann::Index<rtflann::L2<float> > *)index_;
                index->addPoints(point, 0);
                // Rebuild index if it doubles in size
                if (index->sizeAtBuild() * 2 < index->size() + index->removedCount())
                {
                    // clean not used features
                    for (std::list<int>::iterator iter = removedIndexes_.begin(); iter != removedIndexes_.end(); ++iter)
                    {
                        addedDescriptors_.erase(*iter);
                    }
                    removedIndexes_.clear();
                    index->buildIndex();
                }
            }
        }

        addedDescriptors_.insert(std::make_pair(nextIndex_, feature));

        return nextIndex_++;
    }

    void removePoint(unsigned int index)
    {
        if (!index_)
        {
            UERROR("Flann index not yet created!");
            return;
        }

        // If a Segmentation fault occurs in removePoint(), verify that you have this fix in your installed "flann/algorithms/nn_index.h":
        // 707 - if (ids_[id]==id) {
        // 707 + if (id < ids_.size() && ids_[id]==id) {
        // ref: https://github.com/mariusmuja/flann/commit/23051820b2314f07cf40ba633a4067782a982ff3#diff-33762b7383f957c2df17301639af5151

        if (featuresType_ == CV_8UC1)
        {
            ((rtflann::Index<rtflann::Hamming<unsigned char> > *)index_)->removePoint(index);
        }
        else if (useDistanceL1_)
        {
            ((rtflann::Index<rtflann::L1<float> > *)index_)->removePoint(index);
        }
        else
        {
            ((rtflann::Index<rtflann::L2<float> > *)index_)->removePoint(index);
        }

        removedIndexes_.push_back(index);
    }

    void knnSearch(
        const cv::Mat &query,
        cv::Mat &indices,
        cv::Mat &dists,
        int knn,
        const rtflann::SearchParams &params = rtflann::SearchParams())
    {
        if (!index_)
        {
            UERROR("Flann index not yet created!");
            return;
        }
        indices.create(query.rows, knn, CV_32S);
        dists.create(query.rows, knn, featuresType_ == CV_8UC1 ? CV_32S : CV_32F);

        rtflann::Matrix<int> indicesF((int *)indices.data, indices.rows, indices.cols);

        if (featuresType_ == CV_8UC1)
        {
            rtflann::Matrix<unsigned int> distsF((unsigned int *)dists.data, dists.rows, dists.cols);
            rtflann::Matrix<unsigned char> queryF(query.data, query.rows, query.cols);
            ((rtflann::Index<rtflann::Hamming<unsigned char> > *)index_)->knnSearch(queryF, indicesF, distsF, knn, params);
        }
        else
        {
            rtflann::Matrix<float> distsF((float *)dists.data, dists.rows, dists.cols);
            rtflann::Matrix<float> queryF((float *)query.data, query.rows, query.cols);
            if (useDistanceL1_)
            {
                ((rtflann::Index<rtflann::L1<float> > *)index_)->knnSearch(queryF, indicesF, distsF, knn, params);
            }
            else
            {
                ((rtflann::Index<rtflann::L2<float> > *)index_)->knnSearch(queryF, indicesF, distsF, knn, params);
            }
        }
    }

private:
    void *index_;
    unsigned int nextIndex_;
    int featuresType_;
    int featuresDim_;
    bool isLSH_;
    bool useDistanceL1_; // true=EUCLEDIAN_L2 false=MANHATTAN_L1

    // keep feature in memory until the tree is rebuilt
    // (in case the word is deleted when removed from the VWDictFixed)
    std::map<int, cv::Mat> addedDescriptors_;
    std::list<int> removedIndexes_;
};

const int VWDictFixed::ID_START = 1;
const int VWDictFixed::ID_INVALID = 0;

VWDictFixed::VWDictFixed(const ParametersMap &parameters) :
    _totalActiveReferences(0),
    _dictionaryPath(Parameters::defaultKpDictionaryPath()),
    useDistanceL1_(false),
    _flannIndex(new FlannIndex()),
    _strategy(kNNFlannKdTree)
{
    this->parseParameters(parameters);
}

VWDictFixed::~VWDictFixed()
{
    this->clear();
    delete _flannIndex;
}

void VWDictFixed::parseParameters(const ParametersMap &parameters)
{
    ParametersMap::const_iterator iter;

    std::string dictionaryPath = _dictionaryPath;
    if ((iter = parameters.find(Parameters::kKpDictionaryPath())) != parameters.end())
    {
        dictionaryPath = (*iter).second.c_str();
    }

    this->setFixedDictionary(dictionaryPath);
}

void VWDictFixed::setFixedDictionary(const std::string &dictionaryPath)
{
    if (!dictionaryPath.empty())
    {
        if ((!_incrementalDictionary && _dictionaryPath.compare(dictionaryPath) != 0) ||
                _visualWords.size() == 0)
        {
            std::ifstream file;
            file.open(dictionaryPath.c_str(), std::ifstream::in);
            if (file.good())
            {
                UDEBUG("Deleting old dictionary and loading the new one from \"%s\"", dictionaryPath.c_str());
                UTimer timer;

                // first line is the header
                std::string str;
                std::list<std::string> strList;
                std::getline(file, str);
                strList = uSplitNumChar(str);
                unsigned int dimension  = 0;
                for (std::list<std::string>::iterator iter = strList.begin(); iter != strList.end(); ++iter)
                {
                    if (uIsDigit(iter->at(0)))
                    {
                        dimension = std::atoi(iter->c_str());
                        break;
                    }
                }

                if (dimension == 0 || dimension > 1000)
                {
                    UERROR("Invalid dictionary file, visual word dimension (%d) is not valid, \"%s\"", dimension, dictionaryPath.c_str());
                }
                else
                {
                    // Process all words
                    while (file.good())
                    {
                        std::getline(file, str);
                        strList = uSplit(str);
                        if (strList.size() == dimension + 1)
                        {
                            //first one is the visual word id
                            std::list<std::string>::iterator iter = strList.begin();
                            int id = std::atoi(iter->c_str());
                            cv::Mat descriptor(1, dimension, CV_32F);
                            ++iter;
                            unsigned int i = 0;

                            //get descriptor
                            for (; i < dimension && iter != strList.end(); ++i, ++iter)
                            {
                                descriptor.at<float>(i) = uStr2Float(*iter);
                            }
                            if (i != dimension)
                            {
                                UERROR("");
                            }

                            VisualWord *vw = new VisualWord(id, descriptor, 0);
                            _visualWords.insert(_visualWords.end(), std::pair<int, VisualWord *>(id, vw));
                            _notIndexedWords.insert(_notIndexedWords.end(), id);
                        }
                        else
                        {
                            UWARN("Cannot parse line \"%s\"", str.c_str());
                        }
                    }
                    this->update();
                    _incrementalDictionary = false;
                }


                UDEBUG("Time changing dictionary = %fs", timer.ticks());
            }
            else
            {
                UERROR("Cannot open dictionary file \"%s\"", dictionaryPath.c_str());
            }
            file.close();
        }
        else if (!_incrementalDictionary)
        {
            UDEBUG("Dictionary \"%s\" already loaded...", dictionaryPath.c_str());
        }
        else
        {
            UERROR("Cannot change to a fixed dictionary if there are already words (%d) in the incremental one.", _visualWords.size());
        }
    }
    else if (_visualWords.size() == 0)
    {
        _incrementalDictionary = false;
    }
    else if (_incrementalDictionary)
    {
        UWARN("Cannot change to fixed dictionary, %d words already loaded as incremental", (int)_visualWords.size());
    }
    _dictionaryPath = dictionaryPath;
}

void VWDictFixed::update()
{
    ULOGGER_DEBUG("");
    if (!_incrementalDictionary && !_notIndexedWords.size())
    {
        // No need to update the search index if we
        // use a fixed dictionary and the index is
        // already built
        return;
    }

    if (_notIndexedWords.size() || _visualWords.size() == 0 || _removedIndexedWords.size())
    {
        _mapIndexId.clear();
        _mapIdIndex.clear();
        _dataTree = cv::Mat();
        _flannIndex->release();

        if (_visualWords.size())
        {
            UTimer timer;
            timer.start();

            int type;
            if (_visualWords.begin()->second->getDescriptor().type() == CV_8U)
            {
                useDistanceL1_ = true;
                if (_strategy == kNNFlannKdTree)
                {
                    type = CV_32F;
                }
                else
                {
                    UFATAL("");
                }
            }
            else
            {
                type = _visualWords.begin()->second->getDescriptor().type();
            }
            int dim = _visualWords.begin()->second->getDescriptor().cols;

            UASSERT(type == CV_32F || type == CV_8U);
            UASSERT(dim > 0);

            // Create the data matrix
            _dataTree = cv::Mat(_visualWords.size(), dim, type); // SURF descriptors are CV_32F
            std::map<int, VisualWord *>::const_iterator iter = _visualWords.begin();
            for (unsigned int i = 0; i < _visualWords.size(); ++i, ++iter)
            {
                cv::Mat descriptor;
                if (iter->second->getDescriptor().type() == CV_8U)
                {
                    if (_strategy == kNNFlannKdTree)
                    {
                        iter->second->getDescriptor().convertTo(descriptor, CV_32F);
                    }
                    else
                    {
                        UFATAL("");
                    }
                }
                else
                {
                    descriptor = iter->second->getDescriptor();
                }

                UASSERT(descriptor.cols == dim);
                UASSERT(descriptor.type() == type);

                descriptor.copyTo(_dataTree.row(i));
                _mapIndexId.insert(_mapIndexId.end(), std::pair<int, int>(i, iter->second->id()));
                _mapIdIndex.insert(_mapIdIndex.end(), std::pair<int, int>(iter->second->id(), i));
            }

            ULOGGER_DEBUG("_mapIndexId.size() = %d, words.size()=%d, _dim=%d", _mapIndexId.size(), _visualWords.size(), dim);
            ULOGGER_DEBUG("copying data = %f s", timer.ticks());

            if (_strategy ==  kNNFlannKdTree)
            {
                UASSERT_MSG(type == CV_32F, "To use KdTree dictionary, float descriptors are required!");
                _flannIndex->build(_dataTree, rtflann::KDTreeIndexParams(), useDistanceL1_);
            }
            else
            {   
                UFATAL("");
            }

            ULOGGER_DEBUG("Time to create kd tree = %f s", timer.ticks());
        }
        UDEBUG("Dictionary updated! (size=%d added=%d removed=%d)",
               _dataTree.rows, _notIndexedWords.size(), _removedIndexedWords.size());
    }
    else
    {
        UDEBUG("Dictionary has not changed, so no need to update it! (size=%d)", _dataTree.rows);
    }
    _notIndexedWords.clear();
    _removedIndexedWords.clear();
    UDEBUG("");
}

void VWDictFixed::clear(bool printWarningsIfNotEmpty)
{
    ULOGGER_DEBUG("");
    if (printWarningsIfNotEmpty)
    {
        if (_visualWords.size())
        {
            UWARN("Visual dictionary would be already empty here (%d words still in dictionary).", (int)_visualWords.size());
        }
        if (_notIndexedWords.size())
        {
            UWARN("Not indexed words should be empty here (%d words still not indexed)", (int)_notIndexedWords.size());
        }
    }
    for (std::map<int, VisualWord *>::iterator i = _visualWords.begin(); i != _visualWords.end(); ++i)
    {
        delete(*i).second;
    }
    _visualWords.clear();
    _notIndexedWords.clear();
    _removedIndexedWords.clear();
    _totalActiveReferences = 0;
    _dataTree = cv::Mat();
    _mapIndexId.clear();
    _mapIdIndex.clear();
    _unusedWords.clear();
    _flannIndex->release();
    useDistanceL1_ = false;
}

void VWDictFixed::addWordRef(int wordId, int signatureId)
{
    if (signatureId > 0 && wordId > 0)
    {
        VisualWord *vw = 0;
        vw = uValue(_visualWords, wordId, vw);
        if (vw)
        {
            vw->addRef(signatureId);
            _totalActiveReferences += 1;

            _unusedWords.erase(vw->id());
        }
        else
        {
            UERROR("Not found word %d", wordId);
        }
    }
}

void VWDictFixed::removeAllWordRef(int wordId, int signatureId)
{
    VisualWord *vw = 0;
    vw = uValue(_visualWords, wordId, vw);
    if (vw)
    {
        _totalActiveReferences -= vw->removeAllRef(signatureId);
        if (vw->getReferences().size() == 0)
        {
            _unusedWords.insert(std::pair<int, VisualWord *>(vw->id(), vw));
        }
    }
}

std::vector<int> VWDictFixed::findNN(const std::list<VisualWord *> &vws) const
{
    UTimer timer;
    timer.start();

    if (_visualWords.size() && vws.size())
    {
        int type = (*vws.begin())->getDescriptor().type();
        int dim = (*vws.begin())->getDescriptor().cols;

        if (dim != _visualWords.begin()->second->getDescriptor().cols)
        {
            UERROR("Descriptors (size=%d) are not the same size as already added words in dictionary(size=%d)", (*vws.begin())->getDescriptor().cols, dim);
            return std::vector<int>(vws.size(), 0);
        }

        if (type != _visualWords.begin()->second->getDescriptor().type())
        {
            UERROR("Descriptors (type=%d) are not the same type as already added words in dictionary(type=%d)", (*vws.begin())->getDescriptor().type(), type);
            return std::vector<int>(vws.size(), 0);
        }

        // fill the request matrix
        int index = 0;
        VisualWord *vw;
        cv::Mat query(vws.size(), dim, type);
        for (std::list<VisualWord *>::const_iterator iter = vws.begin(); iter != vws.end(); ++iter, ++index)
        {
            vw = *iter;
            UASSERT(vw);

            UASSERT(vw->getDescriptor().cols == dim);
            UASSERT(vw->getDescriptor().type() == type);

            vw->getDescriptor().copyTo(query.row(index));
        }
        ULOGGER_DEBUG("Preparation time = %fs", timer.ticks());

        return findNN(query);
    }
    return std::vector<int>(vws.size(), 0);
}

std::vector<int> VWDictFixed::findNN(const cv::Mat &queryIn) const
{
    UTimer timer;
    timer.start();
    std::vector<int> resultIds(queryIn.rows, 0);
    unsigned int k = 2; // k nearest neighbor

    if (_visualWords.size() && queryIn.rows)
    {
        // verify we have the same features
        int dim = _visualWords.begin()->second->getDescriptor().cols;
        int type = _visualWords.begin()->second->getDescriptor().type();
        UASSERT(type == CV_32F || type == CV_8U);

        if (dim != queryIn.cols)
        {
            UERROR("Descriptors (size=%d) are not the same size as already added words in dictionary(size=%d)", queryIn.cols, dim);
            return resultIds;
        }
        if (type != queryIn.type())
        {
            UERROR("Descriptors (type=%d) are not the same type as already added words in dictionary(type=%d)", queryIn.type(), type);
            return resultIds;
        }

        // now compare with the actual index
        cv::Mat query;
        if (queryIn.type() == CV_8U)
        {
            if (_strategy == kNNFlannKdTree)
            {
                queryIn.convertTo(query, CV_32F);
            }
            else
            {
                UFATAL("");
            }
        }
        else
        {
            query = queryIn;
        }
        dim = 0;
        type = -1;
        if (_dataTree.rows || _flannIndex->isBuilt())
        {
            dim = _flannIndex->isBuilt() ? _flannIndex->featuresDim() : _dataTree.cols;
            type = _flannIndex->isBuilt() ? _flannIndex->featuresType() : _dataTree.type();
            UASSERT(type == CV_32F || type == CV_8U);
        }

        if (dim && dim != query.cols)
        {
            UERROR("Descriptors (size=%d) are not the same size as already added words in dictionary(size=%d)", query.cols, dim);
            return resultIds;
        }

        if (type >= 0 && type != query.type())
        {
            UERROR("Descriptors (type=%d) are not the same type as already added words in dictionary(type=%d)", query.type(), type);
            return resultIds;
        }

        std::vector<std::vector<cv::DMatch> > matches;
        bool bruteForce = false;
        cv::Mat results;
        cv::Mat dists;

        if (_flannIndex->isBuilt() || (!_dataTree.empty() && _dataTree.rows >= (int)k))
        {
            //Find nearest neighbors
            UDEBUG("query.rows=%d ", query.rows);

            if (_strategy == kNNFlannKdTree)
            {
                _flannIndex->knnSearch(query, results, dists, k);
            }
            else
            {
                UFATAL("");
            }

            // In case of binary descriptors
            if (dists.type() == CV_32S)
            {
                cv::Mat temp;
                dists.convertTo(temp, CV_32F);
                dists = temp;
            }
        }
        ULOGGER_DEBUG("Search dictionary time = %fs", timer.ticks());

        std::map<int, int> mapIndexIdNotIndexed;
        std::vector<std::vector<cv::DMatch> > matchesNotIndexed;
        if (_notIndexedWords.size())
        {
            cv::Mat dataNotIndexed = cv::Mat::zeros(_notIndexedWords.size(), query.cols, query.type());
            unsigned int index = 0;
            VisualWord *vw;
            for (std::set<int>::iterator iter = _notIndexedWords.begin(); iter != _notIndexedWords.end(); ++iter, ++index)
            {
                vw = _visualWords.at(*iter);

                cv::Mat descriptor;
                if (vw->getDescriptor().type() == CV_8U)
                {
                    if (_strategy == kNNFlannKdTree)
                    {
                        vw->getDescriptor().convertTo(descriptor, CV_32F);
                    }
                    else
                    {
                        UFATAL("");
                    }
                }
                else
                {
                    descriptor = vw->getDescriptor();
                }

                UASSERT(vw != 0 && descriptor.cols == query.cols && descriptor.type() == query.type());
                vw->getDescriptor().copyTo(dataNotIndexed.row(index));
                mapIndexIdNotIndexed.insert(mapIndexIdNotIndexed.end(), std::pair<int, int>(index, vw->id()));
            }

            // Find nearest neighbor
            ULOGGER_DEBUG("Searching in words not indexed...");
            cv::BFMatcher matcher(query.type() == CV_8U ? cv::NORM_HAMMING : useDistanceL1_ ? cv::NORM_L1 : cv::NORM_L2SQR);
            matcher.knnMatch(query, dataNotIndexed, matchesNotIndexed, dataNotIndexed.rows > 1 ? 2 : 1);
        }
        ULOGGER_DEBUG("Search not yet indexed words time = %fs", timer.ticks());

        for (int i = 0; i < query.rows; ++i)
        {
            std::multimap<float, int> fullResults; // Contains results from the kd-tree search [and the naive search in new words]
            if (!bruteForce && dists.cols)
            {
                for (int j = 0; j < dists.cols; ++j)
                {
                    float d = dists.at<float>(i, j);
                    int id = uValue(_mapIndexId, results.at<int>(i, j));
                    if (d >= 0.0f && id > 0)
                    {
                        fullResults.insert(std::pair<float, int>(d, id));
                    }
                }
            }
            else if (bruteForce && matches.size())
            {
                for (unsigned int j = 0; j < matches.at(i).size(); ++j)
                {
                    float d = matches.at(i).at(j).distance;
                    int id = uValue(_mapIndexId, matches.at(i).at(j).trainIdx);
                    if (d >= 0.0f && id > 0)
                    {
                        fullResults.insert(std::pair<float, int>(d, id));
                    }
                }
            }

            // not indexed..
            if (matchesNotIndexed.size())
            {
                for (unsigned int j = 0; j < matchesNotIndexed.at(i).size(); ++j)
                {
                    float d = matchesNotIndexed.at(i).at(j).distance;
                    int id = uValue(mapIndexIdNotIndexed, matchesNotIndexed.at(i).at(j).trainIdx);
                    if (d >= 0.0f && id > 0)
                    {
                        fullResults.insert(std::pair<float, int>(d, id));
                    }
                    else
                    {
                        break;
                    }
                }
            }

            if (fullResults.size())
            {
                //Just take the nearest if the dictionary is not incremental
                resultIds[i] = fullResults.begin()->second; // Accepted
            }
        }
        ULOGGER_DEBUG("badDist check time = %fs", timer.ticks());
    }
    return resultIds;
}

void VWDictFixed::addWord(VisualWord *vw)
{
    if (vw)
    {
        _visualWords.insert(std::pair<int, VisualWord *>(vw->id(), vw));
        _notIndexedWords.insert(vw->id());
        if (vw->getReferences().size())
        {
            _totalActiveReferences += uSum(uValues(vw->getReferences()));
        }
        else
        {
            _unusedWords.insert(std::pair<int, VisualWord *>(vw->id(), vw));
        }
    }
}

const VisualWord *VWDictFixed::getWord(int id) const
{
    return uValue(_visualWords, id, (VisualWord *)0);
}

VisualWord *VWDictFixed::getUnusedWord(int id) const
{
    return uValue(_unusedWords, id, (VisualWord *)0);
}

std::vector<VisualWord *> VWDictFixed::getUnusedWords() const
{
    if (!_incrementalDictionary)
    {
        ULOGGER_WARN("This method does nothing on a fixed dictionary");
        return std::vector<VisualWord *>();
    }
    return uValues(_unusedWords);
}

void VWDictFixed::removeWords(const std::vector<VisualWord *> &words)
{
    for (unsigned int i = 0; i < words.size(); ++i)
    {
        _visualWords.erase(words[i]->id());
        _unusedWords.erase(words[i]->id());
        if (_notIndexedWords.erase(words[i]->id()) == 0)
        {
            _removedIndexedWords.insert(words[i]->id());
        }
    }
}

void VWDictFixed::deleteUnusedWords()
{
    std::vector<VisualWord *> unusedWords = uValues(_unusedWords);
    removeWords(unusedWords);
    for (unsigned int i = 0; i < unusedWords.size(); ++i)
    {
        delete unusedWords[i];
    }
}
