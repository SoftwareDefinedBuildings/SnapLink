#pragma once

#include <opencv2/core/core.hpp>
#include <flann/flann.hpp>
#include "Words.h"

class WordsKdTree : public Words
{
public:
    WordsKdTree();
    ~WordsKdTree();

    /**
     * Add words
     */
    void addWords(const std::vector<rtabmap::VisualWord *> &words);

    /**
     * get all words
     */
    const std::map<int, rtabmap::VisualWord *> &getWords() const;

    /**
     * find the indices of the nearst neighbors of descriptors
     */
    std::vector<int> findNN(const cv::Mat &descriptors) const;

    /**
     * clear all words 
     */
    void clear();

private:
    void build();

private:
    int _type;
    int _dim;
    std::map<int, rtabmap::VisualWord *> _words; //<id,rtabmap::VisualWord*>
    flann::Index<flann::L2<float> > *_index;
    cv::Mat _dataMat;
    std::map<int , int> _mapIndexId;
};
