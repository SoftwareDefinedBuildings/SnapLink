#include <cassert>
#include "data/SignaturesSimple.h"

void SignaturesSimple::putSignatures(std::list< std::unique_ptr<Signature> > &&signatures)
{
    for (auto & signature : signatures)
    {
        assert(signature != nullptr);
        int id = signature->getId();
        _signatures.insert(std::make_pair(id, std::move(signature)));
    }
}

const std::map<int, std::unique_ptr<Signature> > &SignaturesSimple::getSignatures() const
{
    return _signatures;
}

std::vector<int> SignaturesSimple::findKNN(const std::vector<int> &wordIds, int k) const
{
    std::map<int, float> similarities;
    for (const auto & signature : _signatures)
    {
        float similarity = computeSimilarity(wordIds, *(signature.second));
        similarities.insert(std::make_pair(signature.second->getId(), similarity));
    }

    std::vector<int> topIds;
    if (similarities.size())
    {
        std::vector< std::pair<int, float> > topSimilarities(k);
        std::partial_sort_copy(similarities.begin(), similarities.end(), topSimilarities.begin(), topSimilarities.end(), compareSimilarity);
        for (const auto & similarity : topSimilarities)
        {
            topIds.emplace_back(similarity.first);
        }
    }

    return topIds;
}

float SignaturesSimple::computeSimilarity(const std::vector<int> &wordIds, const Signature &signature)
{
    std::set<int> uniqueWordIds(wordIds.begin(), wordIds.end());
    const std::multimap<int, cv::KeyPoint> &sigWords = signature.getWords();
    float numPairs = 0;
    for (int uniqueWordId : uniqueWordIds)
    {
        auto iter = sigWords.find(uniqueWordId);
        if (iter != sigWords.end())
        {
            numPairs += 1;
        }
    }
    return numPairs;
}

bool SignaturesSimple::compareSimilarity(const std::pair<int, float> &l, const std::pair<int, float> &r)
{
    return l.second > r.second;
}

