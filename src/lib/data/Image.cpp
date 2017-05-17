#include "lib/data/Image.h"

Image::Image(int id, int roomId, const cv::Mat &image, const cv::Mat &depth,
             const Transform &pose, const CameraModel &camera)
    : _id(id), _roomId(roomId), _image(image), _depth(depth), _pose(pose),
      _camera(camera) {}

int Image::getId() const { return _id; }

int Image::getRoomId() const { return _roomId; }

const cv::Mat &Image::getImage() const { return _image; }

const cv::Mat &Image::getDepth() const { return _depth; }

const Transform &Image::getPose() const { return _pose; }

const CameraModel &Image::getCameraModel() const { return _camera; }

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Image::getCloud(int decimation) const {
  // generate a new pcl point cloud and return
  auto cloud = cloudFromDepthRGB(_image, _depth, _camera, decimation, 0, 0);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
  return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
Image::cloudFromDepthRGB(const cv::Mat &imageRgb, const cv::Mat &imageDepthIn,
                         const CameraModel &model, int decimation,
                         float maxDepth, float minDepth) const {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  if (decimation == 0) {
    decimation = 1;
  }
  if (decimation < 0) {
    if (imageRgb.rows % decimation != 0 || imageRgb.cols % decimation != 0) {
      int oldDecimation = decimation;
      while (decimation <= -1) {
        if (imageRgb.rows % decimation == 0 &&
            imageRgb.cols % decimation == 0) {
          break;
        }
        ++decimation;
      }

      if (imageRgb.rows % oldDecimation != 0 ||
          imageRgb.cols % oldDecimation != 0) {
        std::cerr << "Decimation " << oldDecimation
                  << " is not valid for current image size (rgb="
                  << imageRgb.cols << " x " << imageRgb.rows
                  << ". Highest compatible decimation used=" << decimation;
      }
    }
  } else {
    if (imageDepthIn.rows % decimation != 0 ||
        imageDepthIn.cols % decimation != 0) {
      int oldDecimation = decimation;
      while (decimation >= 1) {
        if (imageDepthIn.rows % decimation == 0 &&
            imageDepthIn.cols % decimation == 0) {
          break;
        }
        --decimation;
      }

      if (imageDepthIn.rows % oldDecimation != 0 ||
          imageDepthIn.cols % oldDecimation != 0) {
        std::cerr << "Decimation " << oldDecimation
                  << " is not valid for current image size (rgb="
                  << imageRgb.cols << " x " << imageRgb.rows
                  << ". Highest compatible decimation used=" << decimation;
      }
    }
  }

  cv::Mat imageDepth = imageDepthIn;

  bool mono;
  if (imageRgb.channels() == 3) // BGR
  {
    mono = false;
  } else if (imageRgb.channels() == 1) // Mono
  {
    mono = true;
  } else {
    return cloud;
  }

  // cloud.header = cameraInfo.header;
  cloud->height = imageDepth.rows / decimation;
  cloud->width = imageDepth.cols / decimation;
  cloud->is_dense = false;
  cloud->resize(cloud->height * cloud->width);

  float rgbToDepthFactorX = float(imageRgb.cols) / float(imageDepth.cols);
  float rgbToDepthFactorY = float(imageRgb.rows) / float(imageDepth.rows);
  float depthFx = model.fx() / rgbToDepthFactorX;
  float depthFy = model.fy() / rgbToDepthFactorY;
  float depthCx = model.cx() / rgbToDepthFactorX;
  float depthCy = model.cy() / rgbToDepthFactorY;

  int oi = 0;
  for (int h = 0; h < imageDepth.rows && h / decimation < (int)cloud->height;
       h += decimation) {
    for (int w = 0; w < imageDepth.cols && w / decimation < (int)cloud->width;
         w += decimation) {
      pcl::PointXYZRGB &pt =
          cloud->at((h / decimation) * cloud->width + (w / decimation));

      int x = int(w * rgbToDepthFactorX);
      int y = int(h * rgbToDepthFactorY);
      assert(x >= 0 && x < imageRgb.cols && y >= 0 && y < imageRgb.rows);
      if (!mono) {
        const unsigned char *bgr = imageRgb.ptr<unsigned char>(y, x);
        pt.b = bgr[0];
        pt.g = bgr[1];
        pt.r = bgr[2];
      } else {
        unsigned char v = imageRgb.at<unsigned char>(y, x);
        pt.b = v;
        pt.g = v;
        pt.r = v;
      }

      pcl::PointXYZ ptXYZ = projectDepthTo3D(imageDepth, w, h, depthCx, depthCy,
                                             depthFx, depthFy, false, 0.02f);
      if (pcl::isFinite(ptXYZ) && ptXYZ.z >= minDepth &&
          (maxDepth <= 0.0f || ptXYZ.z <= maxDepth)) {
        pt.x = ptXYZ.x;
        pt.y = ptXYZ.y;
        pt.z = ptXYZ.z;
        ++oi;
      } else {
        pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
      }
    }
  }

  if (oi == 0) {
    std::cerr << "Cloud with only NaN values created!\n";
  }
  return cloud;
}

float Image::getDepth(const cv::Mat &depthImage, float x, float y,
                      bool smoothing, float maxZError,
                      bool estWithNeighborsIfNull) const {
  assert(!depthImage.empty());
  assert(depthImage.type() == CV_16UC1 || depthImage.type() == CV_32FC1);

  int u = int(x + 0.5f);
  int v = int(y + 0.5f);
  if (u == depthImage.cols && x < float(depthImage.cols)) {
    u = depthImage.cols - 1;
  }
  if (v == depthImage.rows && y < float(depthImage.rows)) {
    v = depthImage.rows - 1;
  }

  if (!(u >= 0 && u < depthImage.cols && v >= 0 && v < depthImage.rows)) {
    fprintf(stderr, "!(x >=0 && x<depthImage.cols && y >=0 && "
                    "y<depthImage.rows) cond failed! returning bad point. "
                    "(x=%f (u=%d), y=%f (v=%d), cols=%d, rows=%d)",
            x, u, y, v, depthImage.cols, depthImage.rows);
    return 0;
  }

  bool isInMM = depthImage.type() == CV_16UC1; // is in mm?

  // Inspired from RGBDFrame::getGaussianMixtureDistribution() method from
  // https://github.com/ccny-ros-pkg/rgbdtools/blob/master/src/rgbd_frame.cpp
  // Window weights:
  //  | 1 | 2 | 1 |
  //  | 2 | 4 | 2 |
  //  | 1 | 2 | 1 |
  int u_start = std::max(u - 1, 0);
  int v_start = std::max(v - 1, 0);
  int u_end = std::min(u + 1, depthImage.cols - 1);
  int v_end = std::min(v + 1, depthImage.rows - 1);

  float depth = 0.0f;
  if (isInMM) {
    if (depthImage.at<unsigned short>(v, u) > 0 &&
        depthImage.at<unsigned short>(v, u) <
            std::numeric_limits<unsigned short>::max()) {
      depth = float(depthImage.at<unsigned short>(v, u)) * 0.001f;
    }
  } else {
    depth = depthImage.at<float>(v, u);
  }

  if ((depth == 0.0f || !std::isfinite(depth)) && estWithNeighborsIfNull) {
    // all cells no2 must be under the zError to be accepted
    float tmp = 0.0f;
    int count = 0;
    for (int uu = u_start; uu <= u_end; ++uu) {
      for (int vv = v_start; vv <= v_end; ++vv) {
        if ((uu == u && vv != v) || (uu != u && vv == v)) {
          float d = 0.0f;
          if (isInMM) {
            if (depthImage.at<unsigned short>(vv, uu) > 0 &&
                depthImage.at<unsigned short>(vv, uu) <
                    std::numeric_limits<unsigned short>::max()) {
              d = float(depthImage.at<unsigned short>(vv, uu)) * 0.001f;
            }
          } else {
            d = depthImage.at<float>(vv, uu);
          }
          if (d != 0.0f && std::isfinite(d)) {
            if (tmp == 0.0f) {
              tmp = d;
              ++count;
            } else if (fabs(d - tmp / float(count)) < maxZError) {
              tmp += d;
              ++count;
            }
          }
        }
      }
    }
    if (count > 1) {
      depth = tmp / float(count);
    }
  }

  if (depth != 0.0f && std::isfinite(depth)) {
    if (smoothing) {
      float sumWeights = 0.0f;
      float sumDepths = 0.0f;
      for (int uu = u_start; uu <= u_end; ++uu) {
        for (int vv = v_start; vv <= v_end; ++vv) {
          if (!(uu == u && vv == v)) {
            float d = 0.0f;
            if (isInMM) {
              if (depthImage.at<unsigned short>(vv, uu) > 0 &&
                  depthImage.at<unsigned short>(vv, uu) <
                      std::numeric_limits<unsigned short>::max()) {
                d = float(depthImage.at<unsigned short>(vv, uu)) * 0.001f;
              }
            } else {
              d = depthImage.at<float>(vv, uu);
            }

            // ignore if not valid or depth difference is too high
            if (d != 0.0f && std::isfinite(d) && fabs(d - depth) < maxZError) {
              if (uu == u || vv == v) {
                sumWeights += 2.0f;
                d *= 2.0f;
              } else {
                sumWeights += 1.0f;
              }
              sumDepths += d;
            }
          }
        }
      }
      // set window weight to center point
      depth *= 4.0f;
      sumWeights += 4.0f;

      // mean
      depth = (depth + sumDepths) / sumWeights;
    }
  } else {
    depth = 0;
  }
  return depth;
}

pcl::PointXYZ Image::projectDepthTo3D(const cv::Mat &depthImage, float x,
                                      float y, float cx, float cy, float fx,
                                      float fy, bool smoothing,
                                      float maxZError) const {

  pcl::PointXYZ pt;

  float depth = getDepth(depthImage, x, y, smoothing, maxZError, false);
  if (depth > 0.0f) {
    // Use correct principal point from calibration
    cx = cx > 0.0f ? cx
                   : static_cast<float>(depthImage.cols / 2) - 0.5f; // cameraInfo.K.at(2)
    cy = cy > 0.0f ? cy
                   : static_cast<float>(depthImage.rows / 2) - 0.5f; // cameraInfo.K.at(5)

    // Fill in XYZ
    pt.x = (x - cx) * depth / fx;
    pt.y = (y - cy) * depth / fy;
    pt.z = depth;
  } else {
    pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
  }
  return pt;
}
