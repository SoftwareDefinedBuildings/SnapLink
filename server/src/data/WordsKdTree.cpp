#include <cassert>
#include "data/WordsKdTree.h"

WordsKdTree::WordsKdTree() :
    _type(-1),
    _dim(-1)
{
}

void WordsKdTree::putWords(std::list< std::unique_ptr<rtabmap::VisualWord> > &&words)
{
    std::move(words.begin(), words.end(), std::back_inserter(_words));
    build();
}

std::vector<int> WordsKdTree::findNNs(const cv::Mat &descriptors) const
{
    std::vector<int> resultIds(descriptors.rows, 0);

    if (_words.size() && descriptors.rows)
    {
        // verify we have the same features
        assert(_type == descriptors.type());
        assert(_dim == descriptors.cols);

        cv::Mat indices;
        cv::Mat dists;
        if (_index != nullptr)
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
            cv::Mat descriptor = (*_words.begin())->getDescriptor();
            _type = descriptor.type();
            _dim = descriptor.cols;
        }

        // Create the data matrix
        _dataMat = cv::Mat(_words.size(), _dim, _type);
        int i = 0;
        for (const auto & word : _words)
        {
            cv::Mat descriptor = word->getDescriptor();

            assert(descriptor.type() == _type);
            assert(descriptor.cols == _dim);

            descriptor.copyTo(_dataMat.row(i));
            _mapIndexId.insert(_mapIndexId.end(), std::make_pair(i, word->id()));
            i++;
        }

        flann::Matrix<float> dataset((float *)_dataMat.data, _dataMat.rows, _dataMat.cols);
        _index.reset(new flann::Index< flann::L2<float> >(dataset, flann::KDTreeIndexParams(4)));
        _index->buildIndex();
    }
}
