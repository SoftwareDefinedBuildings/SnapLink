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

    virtual void parseParameters(const rtabmap::ParametersMap &parameters);

    virtual void update();

    virtual void addWord(rtabmap::VisualWord *vw);

    std::vector<int> findNN(const std::list<rtabmap::VisualWord *> &vws) const;
    std::vector<int> findNN(const cv::Mat &descriptors) const;

    void addWordRef(int wordId, int signatureId);
    void removeAllWordRef(int wordId, int signatureId);
    const rtabmap::VisualWord *getWord(int id) const;
    rtabmap::VisualWord *getUnusedWord(int id) const;
    const std::map<int, rtabmap::VisualWord *> &getVisualWords() const
    {
        return _visualWords;
    }
    int getTotalActiveReferences() const
    {
        return _totalActiveReferences;
    }
    void setFixedDictionary(const std::string &dictionaryPath);

    void clear(bool printWarningsIfNotEmpty = true);
    std::vector<rtabmap::VisualWord *> getUnusedWords() const;
    unsigned int getUnusedWordsSize() const
    {
        return (int)_unusedWords.size();
    }
    void removeWords(const std::vector<rtabmap::VisualWord *> &words); // caller must delete the words
    void deleteUnusedWords();

protected:
    std::map<int, rtabmap::VisualWord *> _visualWords; //<id,rtabmap::VisualWord*>
    int _totalActiveReferences; // keep track of all references for updating the common signature

private:
    std::string _dictionaryPath; // a pre-computed dictionary (.txt)
    bool useDistanceL1_;
    FlannIndex *_flannIndex;
    cv::Mat _dataTree;
    NNStrategy _strategy;
    std::map<int , int> _mapIndexId;
    std::map<int , int> _mapIdIndex;
    std::map<int, rtabmap::VisualWord *> _unusedWords; //<id,rtabmap::VisualWord*>, note that these words stay in _visualWords
    std::set<int> _notIndexedWords; // Words that are not indexed in the dictionary
    std::set<int> _removedIndexedWords; // Words not anymore in the dictionary but still indexed in the dictionary
};
