#include "label/widget.h"
#include "lib/data/Transform.h"
#include "lib/util/Utility.h"
#include "ui_widget.h"
#include <QImage>
#include <QMouseEvent>
#include <QPainter>
#include <QPixmap>
#include <QPoint>
#include <iostream>
#include <rtabmap/core/CameraRGBD.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/Optimizer.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/RtabmapThread.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/utilite/UConversion.h>
#include <string>
#include "lib/data/Image.h"
#include "lib/data/Transform.h"
#include <pcl/common/transforms.h>

Widget::Widget(QWidget *parent) : QWidget(parent), _ui(new Ui::Widget) {
  _ui->setupUi(this);


  // connect signals and slots
  connect(_ui->slider, SIGNAL(valueChanged(int)), this,
          SLOT(setSliderValue(int)));
  connect(_ui->pushButton, SIGNAL(released()), this, SLOT(saveLabel()));
}

Widget::~Widget() {
}

bool Widget::init(std::string path) {
  std::set<std::string> dbFiles{path};
  if (!_adapter.init(dbFiles)) {
    std::cerr << "reading data failed";
    return 1;
  }

  const std::map<int, std::vector<Image>> &images = _adapter.getImages();
  assert(images.size() == 1);
  numImages = images.begin()->second.size();

  if (numImages <= 0) {
    std::cerr << "Database does not have any images" << std::endl;
    return false;
  }

  _ui->slider->setRange(1, numImages);

  showImage(1);

  setLabel("enter label name");

  return true;
}


void Widget::setSliderValue(int value) {
  _ui->label_id->setText(QString::number(value));
  showImage(value);
}

void Widget::saveLabel() {
  std::string label_name = _ui->lineEdit_label->text().toStdString();
  std::string label_id = _ui->label_id->text().toStdString();
  std::string label_x = _ui->label_x->text().toStdString();
  std::string label_y = _ui->label_y->text().toStdString();

  if (label_name.length() == 0) {
    std::string msg = "Label name empty";
    UWARN(msg.c_str());
    _ui->label_status->setText(msg.c_str());
    return;
  }

  //In the case of labeling tool, only 1 room is opened in adapter
  int roomId = 0;
  if (!_adapter.putLabel(roomId, label_name, label_id, label_x, label_y)) {
    std::string msg = "Could not convert label or save label to label table";
    UWARN(msg.c_str());
    _ui->label_status->setText(msg.c_str());
  } else {
    _ui->label_status->setText("Saved label to database");

    // display saved label on UI
    projectPoints();
  }
}

void Widget::showImage(int index) {
  const std::map<int, std::vector<Image>> &images = _adapter.getImages();
  assert(images.size() == 1);
  Image image = images.begin()->second[0];
  for (const auto &singleImage : images.begin()->second) {
    if(singleImage.getId() == index) {
      image = singleImage;
      break;
    }
  }
  cv::Mat raw = image.getImage();
  QImage qImage =
      QImage(raw.data, raw.cols, raw.rows, raw.step, QImage::Format_RGB888);
  QPixmap pixmap =
      QPixmap::fromImage(qImage.rgbSwapped()); // need to change BGR -> RGBz

  _ui->label_img->setPixmap(pixmap);

  // display saved labels
  projectPoints();
}

void Widget::showLabel(int x, int y, std::string label) {
  // get Pixmap drawn on UI
  QPixmap *imagePixmap = (QPixmap *)_ui->label_img->pixmap();
  QPainter painter(imagePixmap);

  // superimpose label dot on image
  QImage dot(":/square.jpg");
  QPixmap dotmap = QPixmap::fromImage(dot);

  // draw dot at center of label point
  int drawX = x - (dot.width() / 2);
  int drawY = y - (dot.height() / 2);
  painter.drawPixmap(drawX, drawY, dotmap);

  painter.setFont(QFont("Arial", 8, QFont::Bold));
  QPen penHText(QColor(0, 255, 0)); // green
  painter.setPen(penHText);
  painter.drawText(QPoint(drawX, drawY), label.c_str());

  // update image displayed on UI
  _ui->label_img->setPixmap(*imagePixmap);
}

void Widget::mousePressEvent(QMouseEvent *event) {
  const QPoint p = event->pos();
  _ui->label_x->setText(QString::number(p.x()));
  _ui->label_y->setText(QString::number(p.y()));

  int imageId = _ui->slider->value();
  cv::Point3f pWorld;
  cv::Point2f xy(p.x(), p.y()); 
  Image image = _adapter.getImages().at(0)[imageId];
  if (Utility::getPoint3World(image, xy, pWorld)) {
    _ui->label_status->setText("3D conversion success!");
    _ui->pushButton->setEnabled(true);
  } else {
    _ui->label_status->setText("Failed to convert");
    _ui->pushButton->setEnabled(false);
  }

  // display saved labels
  projectPoints();
}

void Widget::setLabel(const QString &name) {
  _ui->lineEdit_label->setText(name);
}

QString Widget::getLabel() const { return _ui->lineEdit_label->text(); }

/* Project points on image */
void Widget::projectPoints() {
  // get list of points
  std::vector<cv::Point3f> points;
  std::vector<std::string> labels;
  if (!getLabels(points, labels) || points.size() == 0 ||
      points.size() != labels.size()) {
    return;
  }

  int sliderVal = _ui->slider->value();

  // get pose
  Transform pose;
  pose = _adapter.getImages().at(0)[sliderVal].getPose();

  if (pose.isNull()) {
    return;
  }
  
  const std::map<int, std::vector<Image>> &images = _adapter.getImages();
  assert(images.size() == 1);
  Image image = images.begin()->second[0];
  for (const auto &singleImage : images.begin()->second) {
    if(singleImage.getId() == sliderVal) {
      image = singleImage;
      break;
    }
  }
 
  cv::Mat raw = image.getImage(); 
  std::vector<cv::Point2f> planePoints;
  const CameraModel &model = image.getCameraModel();
  cv::Mat K = model.K();
  Transform P = pose.inverse();
  cv::Mat R =
      (cv::Mat_<double>(3, 3) << (double)P.r11(), (double)P.r12(),
       (double)P.r13(), (double)P.r21(), (double)P.r22(), (double)P.r23(),
       (double)P.r31(), (double)P.r32(), (double)P.r33());
  cv::Mat rvec(1, 3, CV_64FC1);
  cv::Rodrigues(R, rvec);
  cv::Mat tvec =
      (cv::Mat_<double>(1, 3) << (double)P.x(), (double)P.y(), (double)P.z());

  // do the projection
  cv::projectPoints(points, rvec, tvec, K, cv::Mat(), planePoints);

  Transform worldP(P.r11(), P.r12(), P.r13(), P.x(), //
                   P.r21(), P.r22(), P.r23(), P.y(), //
                   P.r31(), P.r32(), P.r33(), P.z());

  for (unsigned int i = 0; i < planePoints.size(); i++) {
    cv::Point2f point = planePoints[i];
    std::string label = labels.at(i);
    if (point.x < 0 || point.x > raw.rows || point.y < 0 ||
        point.y > raw.cols ||
        !Utility::isInFrontOfCamera(points[i], worldP)) {
      continue;
    }

    // draw label on UI
    showLabel((int)point.x, (int)point.y, label);
  }
}

/* Get all points in Labels table */
bool Widget::getLabels(std::vector<cv::Point3f> &points,
                       std::vector<std::string> &labels) {
  
  const std::map<int, std::vector<Label>> &dbLabels = _adapter.getLabels();
  if(dbLabels.size()!=1) {
    return false;
  }
  
  for(const auto &label : dbLabels.begin()->second) {
    points.push_back(label.getPoint3());
    labels.push_back(label.getName());
    UDEBUG("Read point (%lf,%lf,%lf) with label %s", label.getPoint3().x, label.getPoint3().y,
           label.getPoint3().z, label.getName());
  } 
  return true;
}

