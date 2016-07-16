#pragma once

#include <rtabmap/core/VisualWord.h>
#include <vector>
#include <list>

class Words
{
public:
    /**
     * Add words, ownership transfer
     */
    virtual void addWords(const std::list<rtabmap::VisualWord *> &words) = 0;

    /**
     * get all words
     * TODO: use map for lookup, there is no point returning as a list. Or maybe only expose a lookup method.
     */
    virtual const std::list<rtabmap::VisualWord *> &getWords() const = 0;

    /**
     * find the indices of the nearst neighbors of descriptors
     */
    virtual std::vector<int> findNNs(const cv::Mat &descriptors) const = 0;
};
