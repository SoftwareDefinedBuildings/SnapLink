#pragma once

#include "lib/adapter/rtabmap/RTABMapAdapter.h"
#include <QWidget>
#include <opencv2/core/core.hpp>
#include <pcl/point_types.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Memory.h>
#include <sqlite3.h>

namespace Ui {
class MeasureWidget;
}

class MeasureWidget : public QWidget {
  Q_OBJECT

public:
  explicit MeasureWidget(QWidget *parent = 0);
  ~MeasureWidget();

  bool init(std::string path);


private slots:
  void setSlider1Value(int);
  void setSlider2Value(int);


protected:
  bool eventFilter(QObject *obj, QEvent *event);
private:
  void mouseClicked(int windowId, QMouseEvent *event);
  void showImage(int windowId, int imageId);
  void showDistance();


private:
  std::unique_ptr<Ui::MeasureWidget> _ui;
  cv::Point3f _pWorld1;
  cv::Point3f _pWorld2;
  bool _pWorldValid1, _pWorldValid2;

  std::string _path;
  int numImages;
  RTABMapAdapter _adapter;
};
