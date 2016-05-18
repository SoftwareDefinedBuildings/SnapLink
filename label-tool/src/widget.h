#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <opencv2/core/core.hpp>
#include <pcl/point_types.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Memory.h>
#include <sqlite3.h>

#include "../../server/src/Localization.h"
#include "../../server/src/MemoryLoc.h"

namespace Ui
{
class Widget;
}

class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = 0);
    ~Widget();

    void setDbPath(std::string path);
    bool openDatabase(void);
    void createLabelTable(void);
    bool setSliderRange(void);

    void setLabel(const QString &name);
    QString getLabel() const;

private slots:
    void setSliderValue(int);
    void saveLabel();
    void mousePressEvent(QMouseEvent *);

private:
    int numImages;

    void showImage(int index);
    void showLabel(int x, int y, std::string label);
    void projectPoints(void);
    bool getLabels(std::vector<cv::Point3f> &points, std::vector<std::string> &labels);
    bool getPoint3World(int imageId, int x, int y, pcl::PointXYZ &pWorld);

private:
    Ui::Widget *_ui;

    sqlite3 *_db;
    std::string _dbPath;
    rtabmap::DBDriver *_dbDriver;
    MemoryLoc _memory;
};

#endif // WIDGET_H
