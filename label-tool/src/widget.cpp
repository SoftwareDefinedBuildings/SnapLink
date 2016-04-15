#include "widget.h"
#include "ui_widget.h"
#include <iostream>
#include <QImage>
#include <QMouseEvent>
#include <QPixmap>
#include <QPoint>
#include <rtabmap/core/CameraRGBD.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/Optimizer.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/utilite/UEventsManager.h>

#include <rtabmap/core/RtabmapThread.h>
#include <rtabmap/core/Parameters.h>

Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);

    // setup DBDriver
    dbDriver = rtabmap::DBDriver::create();

    // connect signals and slots
    connect(ui->slider, SIGNAL (valueChanged(int)), this, SLOT (setSliderValue(int)));
}

Widget::~Widget()
{
    delete ui;
}

void Widget::setDbPath(char *name)
{
    dbPath = QString::fromUtf8(name);
}

bool Widget::openDatabase()
{
    if (!dbDriver->openConnection(dbPath.toStdString()))
    {
        UERROR("Could not open database");
        return false;
    }
    if (!memory.init(dbPath.toStdString(), false))
    {
        UERROR("Error init memory");
        return false;
    }

    return true;
}

bool Widget::setSliderRange()
{
    // get number of images in database
    std::set<int> ids;
    dbDriver->getAllNodeIds(ids);
    numImages = ids.size();

    if (numImages <= 0)
    {
        return false;
    }

    // image ID is 1-indexed
    ui->slider->setRange(1, numImages);

    // display first image
    showImage(1);

    return true;
}

void Widget::setSliderValue(int value)
{
    std::stringstream stream;
    stream << value << " out of " << numImages;

    ui->label_id->setText(QString::fromStdString(stream.str()));
    showImage(value);
}

void Widget::showImage(int index)
{
    rtabmap::SensorData data;
    dbDriver->getNodeData(index, data);
    data.uncompressData();
    cv::Mat raw = data.imageRaw();

    QImage image = QImage(raw.data, raw.cols, raw.rows, raw.step, QImage::Format_RGB888);
    QPixmap pixmap = QPixmap::fromImage(image.rgbSwapped()); // need to change BGR -> RGBz

    ui->imgLabel->setPixmap(pixmap);
}

void Widget::mousePressEvent(QMouseEvent *event)
{
    const QPoint p = event->pos();
    ui->label_x->setText(QString::number(p.x()));
    ui->label_y->setText(QString::number(p.y()));

    convertTo3D(ui->slider->value(), p.x(), p.y());
}

bool Widget::convertTo3D(int imageId, int x, int y)
{
    std::map<int, rtabmap::Transform> optimizedPoses = optimizeGraph(memory);
    pcl::PointXYZ pWorld;
    if (convert(imageId, x, y, memory, optimizedPoses, pWorld))
    {
        ui->label_status->setText("3D conversion success!");
    }
    else
    {
        ui->label_status->setText("Failed to convert");
    }
}

/* convert() and optimizeGraph() taken from label_tool/src/main.cpp */
bool Widget::convert(int imageId, int x, int y, rtabmap::Memory &memory, std::map<int, rtabmap::Transform> &poses, pcl::PointXYZ &pWorld)
{
    const rtabmap::SensorData &data = memory.getNodeData(imageId, true);
    const rtabmap::CameraModel &cm = data.cameraModels()[0];
    bool smoothing = false;

    pcl::PointXYZ pLocal = rtabmap::util3d::projectDepthTo3D(data.depthRaw(), x, y, cm.cx(), cm.cy(), cm.fx(), cm.fy(), smoothing);
    if (std::isnan(pLocal.x) || std::isnan(pLocal.y) || std::isnan(pLocal.z))
    {
        UWARN("Depth value not valid");
        return false;
    }
    std::map<int, rtabmap::Transform>::const_iterator iter = poses.find(imageId);
    if (iter == poses.end() || iter->second.isNull())
    {
        UWARN("Image pose not found or is Null");
        return false;
    }
    rtabmap::Transform poseWorld = iter->second;
    poseWorld = poseWorld * cm.localTransform();
    pWorld = rtabmap::util3d::transformPoint(pLocal, poseWorld);
    return true;
}

std::map<int, rtabmap::Transform> Widget::optimizeGraph(rtabmap::Memory &memory)
{
    if (memory.getLastWorkingSignature())
    {
        // Get all IDs linked to last signature (including those in Long-Term Memory)
        std::map<int, int> ids = memory.getNeighborsId(memory.getLastWorkingSignature()->id(), 0);

        UINFO("Optimize poses, ids.size() = %d", ids.size());

        // Get all metric constraints (the graph)
        std::map<int, rtabmap::Transform> poses;
        std::multimap<int, rtabmap::Link> links;
        memory.getMetricConstraints(uKeysSet(ids), poses, links);

        // Optimize the graph
        rtabmap::Optimizer::Type optimizerType = rtabmap::Optimizer::kTypeTORO; // options: kTypeTORO, kTypeG2O, kTypeGTSAM, kTypeCVSBA
        rtabmap::Optimizer *graphOptimizer = rtabmap::Optimizer::create(optimizerType);
        std::map<int, rtabmap::Transform> optimizedPoses = graphOptimizer->optimize(poses.begin()->first, poses, links);
        delete graphOptimizer;

        return optimizedPoses;
    }
}

void Widget::setLabel(const QString &name)
{
    ui->lineEdit->setText(name);
}

QString Widget::getLabel() const
{
    return ui->lineEdit->text();
}
