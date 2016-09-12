#pragma once

#include "data/Signatures.h"
#include <memory>

#define TOP_K 1

class SignatureSearch {
public:
  SignatureSearch(const std::shared_ptr<Signatures> &signatures);

  std::vector<int> search(const std::vector<int> &wordIds) const;

private:
  std::shared_ptr<Signatures> _signatures;
};
