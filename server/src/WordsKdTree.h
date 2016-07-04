#pragma once

#include <rtabmap/core/VisualWord.h>
#include <rtabmap/core/Parameters.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <list>
#include <set>

class FlannIndex;

class WordsKdTree : public Words
{
public:
    enum NNStrategy
    {
        kNNFlannNaive,
        kNNFlannKdTree,
        kNNFlannLSH,
        kNNBruteForce,
        kNNBruteForceGPU,
        kNNUndef
    };
    static const int ID_START;
    static const int ID_INVALID;

public:
    VWDictFixed(const rtabmap::ParametersMap &parameters = rtabmap::ParametersMap());
    virtual ~VWDictFixed();

    /**
     * Add words
     */
    virtual void addWords(std::vector<rtabmap::VisualWord *> vw) = 0;

    /**
     * get all words
     */
    virtual const std::map<int, rtabmap::VisualWord *> &getWords() const = 0;

    /**
     * find the indices of the nearst neighbors of descriptors
     */
    virtual std::vector<int> findNN(const cv::Mat &descriptors) const = 0;

    /**
     * clear all words 
     */
    virtual void clear() = 0;

private:
    void update();

private:
    std::map<int, rtabmap::VisualWord *> _visualWords; //<id,rtabmap::VisualWord*>
    FlannIndex *_flannIndex;
    cv::Mat _dataTree;
    std::map<int , int> _mapIndexId;
    std::map<int , int> _mapIdIndex;
};
