#pragma once

#include <rtabmap/core/VisualWord.h>
#include <rtabmap/core/Parameters.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <list>
#include <set>

class FlannIndex;

class VWDictFixed
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

    virtual void update();

    virtual void addWord(rtabmap::VisualWord *vw);

    std::vector<int> findNN(const cv::Mat &descriptors) const;

    const rtabmap::VisualWord *getWord(int id) const;
    int getLastWordId()
    {
        return _lastWordId;
    }
    void setLastWordId(int id)
    {
        _lastWordId = id;
    }
    const std::map<int, rtabmap::VisualWord *> &getVisualWords() const
    {
        return _visualWords;
    }
    int getTotalActiveReferences() const
    {
        return _totalActiveReferences;
    }

    void clear(bool printWarningsIfNotEmpty = true);

protected:
    std::map<int, rtabmap::VisualWord *> _visualWords; //<id,rtabmap::VisualWord*>
    int _totalActiveReferences; // keep track of all references for updating the common signature

private:
    int _lastWordId;
    FlannIndex *_flannIndex;
    cv::Mat _dataTree;
    NNStrategy _strategy;
    std::map<int , int> _mapIndexId;
    std::map<int , int> _mapIdIndex;
};
