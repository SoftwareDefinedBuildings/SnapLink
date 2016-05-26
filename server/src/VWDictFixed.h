#pragma once

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <list>
#include <set>
#include <rtabmap/core/Parameters.h>

class DBDriver;
class VisualWord;
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
    VWDictFixed(const ParametersMap &parameters = ParametersMap());
    virtual ~VWDictFixed();

    virtual void parseParameters(const ParametersMap &parameters);

    virtual void update();

    virtual void addWord(VisualWord *vw);

    std::vector<int> findNN(const std::list<VisualWord *> &vws) const;
    std::vector<int> findNN(const cv::Mat &descriptors) const;

    void addWordRef(int wordId, int signatureId);
    void removeAllWordRef(int wordId, int signatureId);
    const VisualWord *getWord(int id) const;
    VisualWord *getUnusedWord(int id) const;
    const std::map<int, VisualWord *> &getVisualWords() const
    {
        return _visualWords;
    }
    int getTotalActiveReferences() const
    {
        return _totalActiveReferences;
    }
    void setFixedDictionary(const std::string &dictionaryPath);

    void clear(bool printWarningsIfNotEmpty = true);
    std::vector<VisualWord *> getUnusedWords() const;
    unsigned int getUnusedWordsSize() const
    {
        return (int)_unusedWords.size();
    }
    void removeWords(const std::vector<VisualWord *> &words); // caller must delete the words
    void deleteUnusedWords();

protected:
    std::map<int, VisualWord *> _visualWords; //<id,VisualWord*>
    int _totalActiveReferences; // keep track of all references for updating the common signature

private:
    std::string _dictionaryPath; // a pre-computed dictionary (.txt)
    bool useDistanceL1_;
    FlannIndex *_flannIndex;
    cv::Mat _dataTree;
    NNStrategy _strategy;
    std::map<int , int> _mapIndexId;
    std::map<int , int> _mapIdIndex;
    std::map<int, VisualWord *> _unusedWords; //<id,VisualWord*>, note that these words stay in _visualWords
    std::set<int> _removedIndexedWords; // Words not anymore in the dictionary but still indexed in the dictionary
};
