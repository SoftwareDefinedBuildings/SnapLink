#pragma once

#include <QSemaphore>
#include <microhttpd.h>
#include <memory>

enum SessionType
{
    GET = 0,
    POST = 1
};

typedef struct
{
    long overall;
    long overall_start;
    long keypoints;
    long keypoints_start;
    long descriptors;
    long descriptors_start;
    long vwd;
    long vwd_start;
    long search;
    long search_start;
    long pnp;
    long pnp_start;
} TimeInfo;

typedef struct
{
    double fx;
    double fy;
    double cx;
    double cy;
} CameraInfo;

typedef struct
{
    enum SessionType sessionType;
    struct MHD_PostProcessor *postProcessor;
    std::vector<char> data;
    CameraInfo cameraInfo;
    TimeInfo timeInfo;
    std::shared_ptr< std::vector<std::string> > names;
    std::shared_ptr<QSemaphore> detected;
    std::shared_ptr<std::string> answerString;
    std::shared_ptr<int> answerCode;
} SessionInfo;
