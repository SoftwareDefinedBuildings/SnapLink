#pragma once

#include <opencv2/core/core.hpp>
#include <flann/flann.hpp>
#include "Words.h"

class WordsKdTree : public Words
{
public:
    WordsKdTree();
    virtual ~WordsKdTree();

    /**
     * Add words
     */
    void addWords(const std::list<rtabmap::VisualWord *> &words);

    /**
     * get all words
     */
    const std::list<rtabmap::VisualWord *> &getWords() const;

    /**
     * find the indices of the nearst neighbors of descriptors
     */
    std::vector<int> findNNs(const cv::Mat &descriptors) const;

private:
    void build();

private:
    int _type;
    int _dim;
    std::list<rtabmap::VisualWord *> _words;
    flann::Index<flann::L2<float> > *_index;
    cv::Mat _dataMat;
    std::map<int , int> _mapIndexId;
};
