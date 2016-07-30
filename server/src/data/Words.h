#pragma once

#include <rtabmap/core/VisualWord.h>
#include <vector>
#include <list>
#include <memory>

class Words
{
public:
    /**
     * Add words, ownership transfer
     */
    virtual void putWords(std::list< std::unique_ptr<rtabmap::VisualWord> > &&words) = 0;

    /**
     * find the indices of the nearst neighbors of descriptors
     */
    virtual std::vector<int> findNNs(const cv::Mat &descriptors) const = 0;
};
