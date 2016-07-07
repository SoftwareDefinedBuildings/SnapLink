#include <cassert>
#include "SignaturesSimple.h"

SignaturesSimple::SignaturesSimple()
{
}

SignaturesSimple::~SignaturesSimple()
{
    for (std::map<int, Signature *>::iterator iter = _signatures.begin(); i != _signatures.end(); ++i)
    {
        delete i->second;
        i->second = NULL;
    }
    _signatures.clear();
}

void SignaturesSimple::addSignatures(const std::list<Signature *> &signatures)
{
    std::list<Signature *>::const_iterator iter = signatures.begin();
    for (; iter != signatures.end(); iter++)
    {
        Signature *signature = *iter;
        if (signature != NULL)
        {
            _signatures.insert(std::pair<int, Signature *>(signature->getId(), signature));
        }
    }
    build();
}

const std::map<int, Signature *> &SignaturesSimple::getSignatures() const
{
    return _signatures;
}

std::vector<int> SignaturesSimple::findKNN(const std::vector<int> wordIds, int k) const
{
    std::map<int, float> similarities;
    similarities.reserve(_signatures.size());
    std::map<int, Signature *>::const_iterator iter = _signatures.begin();
    for (; iter != _signatures.end(); iter++)
    {
        const Signature *signature = iter->second;
        float similarity = computeSimilarity(wordIds, signature);
        similarities.insert(std::pair<int, float>(signature->getId(), similarity));
    }

    std::vector<int> topIds;
    if (similarities.size())
    {
        std::vector< std::pair<int, float> > topSimilarities(k);
        std::partial_sort_copy(similarities.begin(), similarities.end(), topSimilarities.begin(), topSimilarities.end(), compareSimilarity);
        for (std::vector< std::pair<int, float> >::const_iterator iter = topSimilarities.begin(); iter != topSimilarities.end(); iter++)
        {
            topIds.push_back(it->first);
        }
    }

    return topIds;
}

float SignaturesSimple::computeSimilarity(const std::vector<int> &wordIds, const rtabmap::Signature &signature)
{
    std::set<int> uniqueWordIds(wordIds.begin(), wordIds.end());
    const std::multimap<int, cv::KeyPoint> &sigWords = signature.getWords();
    float numPairs = 0;
    for(std::vector<int>::const_iterator iter = uniqueWordIds.begin(); iter != uniqueWordIds.end(); iter++)
    {
        std::multimap<int, cv::KeyPoint>::const_iterator jter = sigWords.find(*iter);
        if (jter != sigWords.end())
        {
            numPairs += 1;
        }
    }
    return numPairs;
}

bool SignaturesSimple::compareSimilarity(std::pair<int, float> const &l, std::pair<int, float> const &r)
{
    return l.second > r.second;
}

