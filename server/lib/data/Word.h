#pragma once

#include <map>
#include <opencv2/core/core.hpp>

class Word {
public:
  Word(int id, const cv::Mat &descriptor, const std::map<int, int> &references);

  int getId() const;
  const cv::Mat &getDescriptor() const;
  const std::map<int, int> &getReferences() const;

private:
  int _id;
  cv::Mat _descriptor;
  std::map<int, int> _references; // (signature id , occurrence in the signature)
};
