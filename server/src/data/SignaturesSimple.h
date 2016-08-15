#pragma once

#include "data/Signatures.h"

class SignaturesSimple : public Signatures {
public:
  /**
   * Add signatures, ownership transfer
   */
  void putSignatures(std::list<std::unique_ptr<Signature>> &&signatures);

  /**
   * get all signatures
   */
  const std::map<int, std::unique_ptr<Signature>> &getSignatures() const;

  /**
   * find the indices of the k most similar signatures
   */
  std::vector<int> findKNN(const std::vector<int> &wordIds, int k) const;

private:
  static float computeSimilarity(const std::vector<int> &wordIds,
                                 const Signature &signature);
  static bool compareSimilarity(const std::pair<int, float> &l,
                                const std::pair<int, float> &r);

private:
  std::map<int, std::unique_ptr<Signature>> _signatures;
};
