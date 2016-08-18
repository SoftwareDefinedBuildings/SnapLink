#pragma once

#include "data/Signatures.h"
#include <QEvent>
#include <QObject>
#include <memory>

#define TOP_K 1

class Perspective;

class SignatureSearch : public QObject {
public:
  SignatureSearch();
  virtual ~SignatureSearch();

  void putSignatures(std::unique_ptr<Signatures> &&signatures);
  void setPerspective(Perspective *perspective);

protected:
  virtual bool event(QEvent *event);

private:
  std::vector<std::unique_ptr<Signature>>
  search(const std::vector<int> &wordIds) const;
  static std::unique_ptr<std::multimap<int, cv::KeyPoint>>
  createWords(const std::vector<int> &wordIds,
              const std::vector<cv::KeyPoint> &keypoints);

private:
  std::unique_ptr<Signatures> _signatures;
  Perspective *_perspective;
};
