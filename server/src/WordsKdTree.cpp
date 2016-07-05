#include <rtabmap/core/Signature.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Parameters.h>
#include <opencv2/opencv_modules.hpp>
#include <flann/flann.hpp>
#include <fstream>
#include <string>
#include <cassert>
#include "WordsKdTree.h"

class FlannIndex
{
public:
    FlannIndex():
        index_(0),
        featuresDim_(0)
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
            delete(flann::Index<flann::L2<float> > *)index_;
            index_ = 0;
        }
    }

    void build(
        const cv::Mat &features,
        const flann::IndexParams &params)
    {
        this->release();
        assert(index_ == 0);
        assert(features.type() == CV_32FC1 || features.type() == CV_8UC1);
        featuresType_ = features.type();
        featuresDim_ = features.cols;

        flann::Matrix<float> dataset((float *)features.data, features.rows, features.cols);
        index_ = new flann::Index<flann::L2<float> >(dataset, params);
        ((flann::Index<flann::L2<float> > *)index_)->buildIndex();
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

    void knnSearch(
        const cv::Mat &query,
        cv::Mat &indices,
        cv::Mat &dists,
        int knn,
        const flann::SearchParams &params = flann::SearchParams())
    {
        if (!index_)
        {
            return;
        }
        indices.create(query.rows, knn, CV_32S);
        dists.create(query.rows, knn, CV_32F);

        flann::Matrix<int> indicesF((int *)indices.data, indices.rows, indices.cols);

        flann::Matrix<float> distsF((float *)dists.data, dists.rows, dists.cols);
        flann::Matrix<float> queryF((float *)query.data, query.rows, query.cols);
        ((flann::Index<flann::L2<float> > *)index_)->knnSearch(queryF, indicesF, distsF, knn, params);
    }

private:
    void *index_;
    int featuresType_;
    int featuresDim_;
};

WordsKdTree::WordsKdTree() :
    _type(-1),
    _dim(-1),
    _flannIndex(new FlannIndex())
{
}

WordsKdTree::~WordsKdTree()
{
    clear();
    delete _flannIndex;
}

void WordsKdTree::addWords(const std::vector<rtabmap::VisualWord *> &words)
{
    std::vector<rtabmap::VisualWord *>::const_iterator iter = words.begin();
    for (; iter != words.end(); iter++)
    {
        rtabmap::VisualWord *word = *iter;
        if (word != NULL)
        {
            _words.insert(std::pair<int, rtabmap::VisualWord *>(word->id(), word));
        }
    }
    build();
}

const std::map<int, rtabmap::VisualWord *> &WordsKdTree::getWords() const
{
    return _words;
}

std::vector<int> WordsKdTree::findNN(const cv::Mat &descriptors) const
{
    std::vector<int> resultIds(descriptors.rows, 0);

    if (_words.size() && descriptors.rows)
    {
        // verify we have the same features
        assert(_type == descriptors.type());
        assert(_dim == descriptors.cols);

        cv::Mat results;
        cv::Mat dists;

        if (_flannIndex->isBuilt() || !_dataMat.empty())
        {
            //Find nearest neighbors
            _flannIndex->knnSearch(descriptors, results, dists, 1);
        }

        assert(dists.rows == descriptors.rows && dists.cols == 1);
        for (int i = 0; i < descriptors.rows; ++i)
        {
            float d = dists.at<float>(i, 0);
            int id = _mapIndexId.at(results.at<int>(i, 0));
            assert(d >= 0.0f && id > 0);
            resultIds[i] = id;
        }
    }
    return resultIds;
}

void WordsKdTree::clear()
{
    for (std::map<int, rtabmap::VisualWord *>::iterator i = _words.begin(); i != _words.end(); ++i)
    {
        delete(*i).second;
    }
    _words.clear();
    _dataMat = cv::Mat();
    _mapIndexId.clear();
    _flannIndex->release();
}

void WordsKdTree::build()
{
    _mapIndexId.clear();
    _dataMat = cv::Mat();
    _flannIndex->release();

    if (_words.size())
    {
        // use the first word to define the type and dim
        if (_type < 0 || _dim < 0)
        {
            cv::Mat descriptor = _words.begin()->second->getDescriptor();
            _type = descriptor.type();
            _dim = descriptor.cols;
        }

        // Create the data matrix
        _dataMat = cv::Mat(_words.size(), _dim, _type);
        std::map<int, rtabmap::VisualWord *>::const_iterator iter = _words.begin();
        for (unsigned int i = 0; i < _words.size(); i++, iter++)
        {
            cv::Mat descriptor = iter->second->getDescriptor();

            assert(descriptor.type() == _type);
            assert(descriptor.cols == _dim);

            descriptor.copyTo(_dataMat.row(i));
            _mapIndexId.insert(_mapIndexId.end(), std::pair<int, int>(i, iter->second->id()));
        }

        _flannIndex->build(_dataMat, flann::KDTreeIndexParams());
    }
}
