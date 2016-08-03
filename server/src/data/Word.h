#pragma once

#include <opencv2/core/core.hpp>

class Word
{
public:
    Word(int id, const cv::Mat &descriptor);

    int getId() const;
    const cv::Mat &getDescriptor() const;

private:
    int _id;
    cv::Mat _descriptor;
};
