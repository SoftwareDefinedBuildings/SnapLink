#include "measure/MeasureWidget.h"
#include "lib/data/Image.h"
#include "lib/data/Transform.h"
#include "lib/data/Transform.h"
#include "lib/util/Utility.h"
#include "ui_measurewidget.h"
#include <QImage>
#include <QMouseEvent>
#include <QPainter>
#include <QPixmap>
#include <QPoint>
#include <iostream>
#include <pcl/common/transforms.h>
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

MeasureWidget::MeasureWidget(QWidget *parent) : QWidget(parent), _ui(new Ui::MeasureWidget) {
  _ui->setupUi(this);

  qApp->installEventFilter(this);

  // connect signals and slots
  connect(_ui->slider1, SIGNAL(valueChanged(int)), this,
          SLOT(setSlider1Value(int)));
  connect(_ui->slider2, SIGNAL(valueChanged(int)), this,
          SLOT(setSlider2Value(int)));
}

MeasureWidget::~MeasureWidget() {}

bool MeasureWidget::init(std::string path) {
  std::set<std::string> dbFiles{path};
  if (!_adapter.init(dbFiles)) {
    std::cerr << "reading data failed";
    return 1;
  }

  const auto &images = _adapter.getImages();
  assert(images.size() == 1);
  numImages = images.at(0).size();

  if (numImages <= 0) {
    std::cerr << "Database does not have any images" << std::endl;
    return false;
  }
  // Image ID starts from 0
  _ui->slider1->setRange(0, numImages - 1);
  _ui->slider2->setRange(0, numImages - 1);


  _ui->id1->setText("0");
  _ui->id2->setText("0");
  showImage(1,0);
  showImage(2,0);

  _pWorldValid1 = false;
  _pWorldValid2 = false;
  showDistance();

  return true;
}


void MeasureWidget::setSlider1Value(int value) {
    _ui->id1->setText(QString::number(value));
    _pWorldValid1 = false;
    showDistance();
    showImage(1, value);
}

void MeasureWidget::setSlider2Value(int value) {
    _ui->id2->setText(QString::number(value));
    _pWorldValid2 = false;
    showDistance();
    showImage(2, value);
}

bool MeasureWidget::eventFilter(QObject *obj, QEvent *event) {
  if ( event->type() != QEvent::MouseButtonPress )
        return false;
  if ( obj == _ui->image1 ) {
    mouseClicked(1,(QMouseEvent*)event);
    return true;
  } else if(obj == _ui->image2) {
    mouseClicked(2,(QMouseEvent*)event);
    return true;
  } else {
    return false;
  }
}

void MeasureWidget::showImage(int windowId, int imageId) {
  const auto &images = _adapter.getImages();
  assert(images.size() == 1);
  const Image &image = images.at(0).at(imageId);
  cv::Mat raw = image.getImage();
  QImage qImage =
      QImage(raw.data, raw.cols, raw.rows, raw.step, QImage::Format_RGB888);
  QPixmap pixmap =
      QPixmap::fromImage(qImage.rgbSwapped()); // need to change BGR -> RGBz

  if(windowId == 1) {
    _ui->image1->setPixmap(pixmap);
  } else {
    _ui->image2->setPixmap(pixmap);
  }
}

void MeasureWidget::mouseClicked(int windowId, QMouseEvent *event) {
  const QPoint p = event->pos();
  if(windowId == 1) {
    _ui->pos2d1->setText("Position in 2D: " + QString::number(p.x()) + " " + QString::number(p.y()));
  
    int imageId = _ui->slider1->value();
    cv::Point3f pWorld;
    cv::Point2f xy(p.x(), p.y());
    Image image = _adapter.getImages().at(0).at(imageId);


    
    cv::Mat raw = image.getImage();
    QImage qImage =
        QImage(raw.data, raw.cols, raw.rows, raw.step, QImage::Format_RGB888);
    QPixmap pixmap =
        QPixmap::fromImage(qImage.rgbSwapped()); // need to change BGR -> RGBz
    QPainter painter;
    painter.begin(&pixmap);
    painter.setBrush(Qt::red);
    painter.drawEllipse(p.x() - 5, p.y() - 5, 10, 10);
    _ui->image1->setPixmap(pixmap);
    if (Utility::getPoint3World(image, xy, pWorld)) {
      _ui->status1->setText("3D conversion success!");
      _ui->pos3d1->setText("Position in 3D: " + QString::number(pWorld.x) + " " + QString::number(pWorld.y) + " " + QString::number(pWorld.z));
      _pWorld1 = pWorld;
      _pWorldValid1 = true;
    } else {
      _ui->status1->setText("Failed to convert");
      _pWorldValid1 = false;    
    }
    showDistance();
  } else {
    const QPoint p = event->pos();
    _ui->pos2d2->setText("Position in 2D: " + QString::number(p.x()) + " " + QString::number(p.y()));
  
    int imageId = _ui->slider2->value();
    cv::Point3f pWorld;
    cv::Point2f xy(p.x(), p.y());
    Image image = _adapter.getImages().at(0).at(imageId);
    cv::Mat raw = image.getImage();
    QImage qImage =
        QImage(raw.data, raw.cols, raw.rows, raw.step, QImage::Format_RGB888);
    QPixmap pixmap =
        QPixmap::fromImage(qImage.rgbSwapped()); // need to change BGR -> RGBz
    QPainter painter;
    painter.begin(&pixmap);
    painter.setBrush(Qt::red);
    painter.drawEllipse(p.x() - 5, p.y() - 5, 10, 10);
    _ui->image2->setPixmap(pixmap);
    if (Utility::getPoint3World(image, xy, pWorld)) {
      _ui->status2->setText("3D conversion success!");
      _ui->pos3d2->setText("Position in 3D: " + QString::number(pWorld.x) + " " + QString::number(pWorld.y) + " " + QString::number(pWorld.z));
      _pWorld2 = pWorld;
      _pWorldValid2 = true;
    } else {
      _ui->status2->setText("Failed to convert");
      _pWorldValid2 = false;    
    }
    showDistance();
  }
}

void MeasureWidget::showDistance() {
  if(_pWorldValid1 && _pWorldValid2) {
    double distance = cv::norm(_pWorld1-_pWorld2);
    _ui->distance->setText("Distance: " + QString::number(distance) + " meters,   "  + QString::number(distance * 39.3701) + " inches");
  } else {
    _ui->distance->setText("Distance: Unavailable because at least one 3D point not located successfully");
  }
}
