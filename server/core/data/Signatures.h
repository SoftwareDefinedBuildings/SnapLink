#pragma once

#include "data/Signature.h"
#include <list>
#include <map>
#include <vector>

class Signatures {
public:
  virtual ~Signatures() = default;

  /**
   * Add signatures, ownership transfer
   */
  virtual void
  putSignatures(std::list<std::unique_ptr<Signature>> &&signatures) = 0;

  /**
   * get all signatures
   */
  virtual const std::map<int, std::unique_ptr<Signature>> &
  getSignatures() const = 0;

  /**
   * find the indices of the k most similar signatures
   */
  virtual std::vector<int> findKNN(const std::vector<int> &wordIds,
                                   int k) const = 0;
};
