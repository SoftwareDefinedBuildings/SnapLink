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
#include <rtabmap/utilite/UEventsManager.h>
#include <string>

Widget::Widget(QWidget *parent) : QWidget(parent), _ui(new Ui::Widget) {
  _ui->setupUi(this);

  // setup DBDriver
  _dbDriver = rtabmap::DBDriver::create();

  // connect signals and slots
  connect(_ui->slider, SIGNAL(valueChanged(int)), this,
          SLOT(setSliderValue(int)));
  connect(_ui->pushButton, SIGNAL(released()), this, SLOT(saveLabel()));
}

Widget::~Widget() {
  delete _ui;
  if (_db) {
    sqlite3_close(_db);
  }
}

bool Widget::init(std::string path) {
  if (sqlite3_open(path.c_str(), &_db) != SQLITE_OK) {
    std::cerr << "Could not open database" << std::endl;
    return false;
  }
  if (!_dbDriver->openConnection(path)) {
    std::cerr << "Could not open database" << std::endl;
    return false;
  }
  createLabelTable();

  if (!_memory.init(path)) {
    std::cerr << "Error init memory" << std::endl;
    return false;
  }

  // get the graph
  std::map<int, int> idMap =
      _memory.getNeighborsId(_memory.getLastWorkingSignature()->id(), 0, 0);
  std::map<int, rtabmap::Transform> poses;
  std::multimap<int, rtabmap::Link> links;
  _memory.getMetricConstraints(uKeysSet(idMap), poses, links);
  std::cout << "poses read : " << idMap.size() << std::endl;

  // optimize the graph
  rtabmap::Optimizer *optimizer =
      rtabmap::Optimizer::create(rtabmap::Optimizer::kTypeTORO);
  std::multimap<int, rtabmap::Link> linksOut;
  optimizer->getConnectedGraph(poses.begin()->first, poses, links,
                               _optimizedPoses, linksOut);
  std::cout << "poses optimized : " << _optimizedPoses.size() << std::endl;

  // get number of images in database
  std::set<int> ids;
  _dbDriver->getAllNodeIds(ids);
  numImages = ids.size();

  if (numImages <= 0) {
    std::cerr << "Database does not have any images" << std::endl;
    return false;
  }

  // image ID is 1-indexed
  _ui->slider->setRange(1, numImages);

  // display first image
  showImage(1);

  setLabel("enter label name");

  return true;
}

void Widget::createLabelTable() {
  std::string query;
  query = "CREATE TABLE IF NOT EXISTS Labels (\n\t"
          "labelName VARCHAR(255),\n\t"
          "imgId INT,\n\t"
          "x INT,\n\t"
          "y INT\n); ";
  int rc = sqlite3_exec(_db, query.c_str(), NULL, NULL, NULL);
  if (rc != SQLITE_OK) {
    UFATAL("Could not create label table");
  }
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

  int imageId, x, y;
  imageId = std::stoi(label_id);
  x = std::stoi(label_x);
  y = std::stoi(label_y);

  // convert again to verify label has depth
  pcl::PointXYZ pWorld;
  if (getPoint3World(imageId, x, y, pWorld)) {
    std::stringstream saveQuery;
    saveQuery << "INSERT INTO Labels VALUES ('" << label_name << "', '"
              << label_id << "', '" << label_x << "', '" << label_y << "');";

    int rc = sqlite3_exec(_db, saveQuery.str().c_str(), NULL, NULL, NULL);
    if (rc != SQLITE_OK) {
      std::string msg = "Could not save label to label table";
      UWARN(msg.c_str());
      _ui->label_status->setText(msg.c_str());
    } else {
      _ui->label_status->setText("Saved label to database");

      // display saved label on UI
      projectPoints();
    }
  } else {
    std::string msg = "Could not convert label";
    UWARN(msg.c_str());
    _ui->label_status->setText(msg.c_str());
  }
}

void Widget::showImage(int index) {
  rtabmap::SensorData data;
  _dbDriver->getNodeData(index, data);
  data.uncompressData();
  cv::Mat raw = data.imageRaw();

  QImage image =
      QImage(raw.data, raw.cols, raw.rows, raw.step, QImage::Format_RGB888);
  QPixmap pixmap =
      QPixmap::fromImage(image.rgbSwapped()); // need to change BGR -> RGBz

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
  pcl::PointXYZ pWorld;
  if (getPoint3World(imageId, p.x(), p.y(), pWorld)) {
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

  // get sensorData for image
  int sliderVal = _ui->slider->value();
  rtabmap::SensorData data;
  _dbDriver->getNodeData(sliderVal, data);
  data.uncompressData();

  // get pose
  rtabmap::Transform pose;
  if (_optimizedPoses.find(data.id()) != _optimizedPoses.end()) {
    pose = _optimizedPoses.at(data.id());
  } else {
    pose = _memory.getOdomPose(data.id(), true);
  }

  if (pose.isNull()) {
    return;
  }

  std::vector<cv::Point2f> planePoints;
  const rtabmap::CameraModel &model = data.cameraModels()[0];
  cv::Mat K = model.K();
  rtabmap::Transform P = (pose * model.localTransform()).inverse();
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

  for (int i = 0; i < planePoints.size(); i++) {
    cv::Point2f point = planePoints[i];
    std::string label = labels.at(i);
    if (point.x < 0 || point.x > data.imageRaw().rows || point.y < 0 ||
        point.y > data.imageRaw().cols ||
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
  sqlite3_stmt *stmt = NULL;
  int rc;

  std::string sql = "SELECT * from Labels";
  rc = sqlite3_prepare(_db, sql.c_str(), -1, &stmt, NULL);
  if (rc != SQLITE_OK) {
    UERROR("Could not read database: %s", sqlite3_errmsg(_db));
    return false;
  }

  while (sqlite3_step(stmt) == SQLITE_ROW) {
    std::string label(
        reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0)));
    int imageId = sqlite3_column_int(stmt, 1);
    int x = sqlite3_column_int(stmt, 2);
    int y = sqlite3_column_int(stmt, 3);
    pcl::PointXYZ pWorld;
    if (getPoint3World(imageId, x, y, pWorld)) {
      points.push_back(cv::Point3f(pWorld.x, pWorld.y, pWorld.z));
      labels.push_back(label);
      UDEBUG("Read point (%lf,%lf,%lf) with label %s", pWorld.x, pWorld.y,
             pWorld.z, label.c_str());
    }
  }

  sqlite3_finalize(stmt);
  return true;
}

/* TODO functions below are copied from server branch, should make functions
 * accessible and stop copying code */
bool Widget::getPoint3World(int imageId, int x, int y, pcl::PointXYZ &pWorld) {
  bool uncompressedData = true;
  rtabmap::SensorData data = _memory.getNodeData(imageId, uncompressedData);
  const rtabmap::CameraModel &cm = data.cameraModels()[0];
  bool smoothing = false;
  pcl::PointXYZ pLocal = rtabmap::util3d::projectDepthTo3D(
      data.depthRaw(), x, y, cm.cx(), cm.cy(), cm.fx(), cm.fy(), smoothing);
  if (std::isnan(pLocal.x) || std::isnan(pLocal.y) || std::isnan(pLocal.z)) {
    UWARN("Depth value not valid");
    return false;
  }

  rtabmap::Transform poseWorld;
  if (_optimizedPoses.find(imageId) != _optimizedPoses.end()) {
    poseWorld = _optimizedPoses.at(imageId);
  } else {
    UWARN("Image not optimized");
    return false;
    // poseWorld = _memory.getOdomPose(imageId, true);
  }

  if (poseWorld.isNull()) {
    UWARN("Image pose is Null");
    return false;
  }
  poseWorld = poseWorld * cm.localTransform();
  pWorld = rtabmap::util3d::transformPoint(pLocal, poseWorld);
  return true;
}
