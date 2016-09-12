#include "algo/SignatureSearch.h"

SignatureSearch::SignatureSearch(
    const std::shared_ptr<Signatures> &signatures) {
  _signatures = signatures;
}

std::vector<int>
SignatureSearch::search(const std::vector<int> &wordIds) const {
  return _signatures->findKNN(wordIds, TOP_K);
}
