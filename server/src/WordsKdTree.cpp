#include <cassert>
#include "WordsKdTree.h"

WordsKdTree::WordsKdTree() :
    _type(-1),
    _dim(-1),
    _index(NULL)
{
}

WordsKdTree::~WordsKdTree()
{
    clear();
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

        cv::Mat indices;
        cv::Mat dists;
        if (_index != NULL && !_dataMat.empty())
        {
            //Find nearest neighbors
            indices.create(descriptors.rows, 1, CV_32S);
            dists.create(descriptors.rows, 1, CV_32F);

            flann::Matrix<int> indicesF((int *)indices.data, indices.rows, indices.cols);
            flann::Matrix<float> distsF((float *)dists.data, dists.rows, dists.cols);
            flann::Matrix<float> queryF((float *)descriptors.data, descriptors.rows, descriptors.cols);
            _index->knnSearch(queryF, indicesF, distsF, 1, flann::SearchParams(128));
        }

        assert(dists.rows == descriptors.rows && dists.cols == 1);
        for (int i = 0; i < descriptors.rows; ++i)
        {
            float d = dists.at<float>(i, 0);
            int id = _mapIndexId.at(indices.at<int>(i, 0));
            assert(d >= 0.0f && id > 0);
            resultIds[i] = id;
        }
    }
    return resultIds;
}

void WordsKdTree::build()
{
    _mapIndexId.clear();
    _dataMat = cv::Mat();

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

        delete _index;
        _index = NULL;
        flann::Matrix<float> dataset((float *)_dataMat.data, _dataMat.rows, _dataMat.cols);
        _index = new flann::Index<flann::L2<float> >(dataset, flann::KDTreeIndexParams(4));
        _index->buildIndex();
    }
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
    delete _index;
    _index = NULL;
    _type = -1;
    _dim = -1;
}

