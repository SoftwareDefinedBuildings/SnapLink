#pragma once

#include <vector>
#include <list>
#include <memory>
#include <opencv2/core/core.hpp>
#include "Word.h"

class Words
{
public:
    /**
     * Add words, ownership transfer
     */
    virtual void putWords(std::list< std::unique_ptr<Word> > &&words) = 0;

    /**
     * find the indices of the nearst neighbors of descriptors
     */
    virtual std::vector<int> findNNs(const cv::Mat &descriptors) const = 0;
};
