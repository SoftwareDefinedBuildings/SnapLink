#pragma once

#include "Signatures.h"

class SignaturesSimple : public Signatures
{
public:
    SignaturesSimple();
    virtual ~SignaturesSimple();

    /**
     * Add signatures, ownership transfer
     */
    void addSignatures(const std::list<Signature *> &signatures);

    /**
     * get all signatures
     */
    const std::map<int, Signature *> &getSignatures() const;

    /**
     * find the indices of the k most similar signatures
     */
    std::vector<int> findKNN(const std::vector<int> wordIds, int k) const;

private:
    static float computeSimilarity(const std::vector<int> &wordIds, const Signature &signature);
    static bool compareSimilarity(const std::pair<int, float> &l, const std::pair<int, float> &r);
    
private:
    std::map<int, Signature *> _signatures;
};
