#include "widget.h"
#include "ui_widget.h"
#include <iostream>
#include <QImage>
#include <QMouseEvent>
#include <QPainter>
#include <QPixmap>
#include <QPoint>
#include <rtabmap/core/CameraRGBD.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/Optimizer.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/RtabmapThread.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <string>

Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);

    // setup DBDriver
    dbDriver = rtabmap::DBDriver::create();

    // connect signals and slots
    connect(ui->slider, SIGNAL (valueChanged(int)), this, SLOT (setSliderValue(int)));
    connect(ui->pushButton, SIGNAL (released()), this, SLOT (saveLabel()));
}

Widget::~Widget()
{
    delete ui;
    if (db)
    {
        sqlite3_close(db);
    }
}

void Widget::setDbPath(char *name)
{
    dbPath = QString::fromUtf8(name);
}

bool Widget::openDatabase()
{
    std::string path = dbPath.toStdString();
    if (sqlite3_open(path.c_str(), &db) != SQLITE_OK)
    {
        UERROR("Could not open database");
        return false;
    }
    if (!dbDriver->openConnection(path))
    {
        UERROR("Could not open database");
        return false;
    }
    createLabelTable();

    if (!memory.init(path, false))
    {
        UERROR("Error init memory");
        return false;
    }

    rtabmap::ParametersMap memoryParams;
    memoryParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpDetectorStrategy(), uNumber2Str(rtabmap::Feature2D::kFeatureSurf)));
    memoryParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisMinInliers(), "4"));
    memoryParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kSURFGpuVersion(), "true"));
    if (!memoryLoc.init(path, memoryParams))
    {
        UERROR("Initializing memory failed");
        return false;
    }

    return true;
}

void Widget::createLabelTable()
{
    std::string query;
    query = "CREATE TABLE IF NOT EXISTS Labels (\n\t" \
        "labelName VARCHAR(255),\n\t" \
        "imgId INT,\n\t" \
        "x INT,\n\t" \
        "y INT\n); ";
    int rc = sqlite3_exec(db, query.c_str(), NULL, NULL, NULL);
    if (rc != SQLITE_OK)
    {
        UWARN("Could not create label table");
    }
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
    ui->label_id->setText(QString::number(value));
    showImage(value);
}

void Widget::saveLabel()
{
    std::string label_name = ui->lineEdit_label->text().toStdString();
    std::string label_id = ui->label_id->text().toStdString();
    std::string label_x = ui->label_x->text().toStdString();
    std::string label_y = ui->label_y->text().toStdString();

    if (label_name.length() == 0)
    {
        UWARN("Label name empty");
        return;
    }

    int imageId, x, y;
    imageId = std::stoi(label_id);
    x = std::stoi(label_x);
    y = std::stoi(label_y);

    // convert again to verify label has depth
    if (convertTo3D(imageId, x, y))
    {
        std::stringstream saveQuery;
        saveQuery << "INSERT INTO Labels VALUES ('" \
                    << label_name << "', '" << label_id << "', '" << label_x << "', '" << label_y << "');";

        std::cout << saveQuery.str() << std::endl;
        int rc = sqlite3_exec(db, saveQuery.str().c_str(), NULL, NULL, NULL);
        if (rc != SQLITE_OK)
        {
            UWARN("Could not save label to label table");
        }
    }
    else
    {
        UWARN("Could not convert label");
    }
}

void Widget::showImage(int index)
{
    rtabmap::SensorData data;
    dbDriver->getNodeData(index, data);
    data.uncompressData();
    cv::Mat raw = data.imageRaw();

    QImage image = QImage(raw.data, raw.cols, raw.rows, raw.step, QImage::Format_RGB888);
    QPixmap pixmap = QPixmap::fromImage(image.rgbSwapped()); // need to change BGR -> RGBz

    ui->label_img->setPixmap(pixmap);

    // display saved labels
    projectPoints();
}

void Widget::addDot(int x, int y)
{
    // get Pixmap drawn on UI
    QPixmap *imagePixmap = (QPixmap*) ui->label_img->pixmap();
    QPainter painter(imagePixmap);

    // superimpose label dot on image
    QImage dot(":/square.jpg");
    QPixmap dotmap = QPixmap::fromImage(dot);
    painter.drawPixmap(x, y, dotmap);

    // update image displayed on UI
    ui->label_img->setPixmap(*imagePixmap);
}

void Widget::mousePressEvent(QMouseEvent *event)
{
    const QPoint p = event->pos();
    ui->label_x->setText(QString::number(p.x()));
    ui->label_y->setText(QString::number(p.y()));

    convertTo3D(ui->slider->value(), p.x(), p.y());

    // display saved labels
    projectPoints();
}

bool Widget::convertTo3D(int imageId, int x, int y)
{
    std::map<int, rtabmap::Transform> optimizedPoses = optimizeGraph(memory);
    pcl::PointXYZ pWorld;
    if (convert(imageId, x, y, memory, optimizedPoses, pWorld))
    {
        ui->label_status->setText("3D conversion success!");
        return true;
    }
    else
    {
        ui->label_status->setText("Failed to convert");
    }
    return false;
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
    ui->lineEdit_label->setText(name);
}

QString Widget::getLabel() const
{
    return ui->lineEdit_label->text();
}

/* Project points on image */
void Widget::projectPoints()
{
    // get list of points
    std::vector<cv::Point3f> points = getPoints();
    if (points.size() == 0)
    {
        UINFO("No points found");
        return;
    }

    // get sensorData for image
    int sliderVal = ui->slider->value();
    rtabmap::SensorData data;
    dbDriver->getNodeData(sliderVal, data);
    data.uncompressData();

    // get pose
    rtabmap::Transform pose = localize(&data);

    std::vector<cv::Point2f> planePoints;
    const rtabmap::CameraModel &model = data.cameraModels()[0];
    cv::Mat K = model.K();
    rtabmap::Transform P = (pose * model.localTransform()).inverse();
    cv::Mat R = (cv::Mat_<double>(3, 3) <<
            (double)P.r11(), (double)P.r12(), (double)P.r13(),
            (double)P.r21(), (double)P.r22(), (double)P.r23(),
            (double)P.r31(), (double)P.r32(), (double)P.r33());
    cv::Mat rvec(1, 3, CV_64FC1);
    cv::Rodrigues(R, rvec);
    cv::Mat tvec = (cv::Mat_<double>(1, 3) <<
                    (double)P.x(), (double)P.y(), (double)P.z());

    // do the projection
    cv::projectPoints(points, rvec, tvec, K, cv::Mat(), planePoints);

    for (int i = 0; i < planePoints.size(); i++)
    {
        cv::Point2f point = planePoints[i];
        if (point.x < 0 || point.x > data.imageRaw().rows ||
            point.y < 0 || point.y > data.imageRaw().cols)
        {
            continue;
        }

        // draw label on UI
        addDot((int) point.x, (int) point.y);
    }
}

/* Get all points in Labels table */
std::vector<cv::Point3f> Widget::getPoints()
{
    std::vector<cv::Point3f> points;

    sqlite3_stmt *stmt = NULL;
    int rc;

    std::string sql = "SELECT * from Labels";
    rc = sqlite3_prepare(db, sql.c_str(), -1, &stmt, NULL);
    if (rc != SQLITE_OK)
    {
        UERROR("Could not read database: %s", sqlite3_errmsg(db));
        return points;
    }

    while (sqlite3_step(stmt) == SQLITE_ROW)
    {
        std::string label(reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0)));
        int imageId = sqlite3_column_int(stmt, 1);
        int x = sqlite3_column_int(stmt, 2);
        int y = sqlite3_column_int(stmt, 3);
        pcl::PointXYZ pWorld;
        if (getPoint3World(imageId, x, y, &memoryLoc, pWorld))
        {
            points.push_back(cv::Point3f(pWorld.x, pWorld.y, pWorld.z));
            UINFO("Read point (%lf,%lf,%lf) with label %s", pWorld.x, pWorld.y, pWorld.z, label.c_str());
        }
    }

    sqlite3_finalize(stmt);
    return points;
}

/* TODO functions below are copied from server branch, should make functions accessible and stop copying code */
bool Widget::getPoint3World(int imageId, int x, int y, const MemoryLoc *memory, pcl::PointXYZ &pWorld)
{
    const rtabmap::Signature *s = memory->getSignature(imageId);
    if (s == NULL)
    {
        UWARN("Signature %d does not exist", imageId);
        return false;
    }
    const rtabmap::SensorData &data = s->sensorData();
    const rtabmap::CameraModel &cm = data.cameraModels()[0];
    bool smoothing = false;
    pcl::PointXYZ pLocal = rtabmap::util3d::projectDepthTo3D(data.depthRaw(), x, y, cm.cx(), cm.cy(), cm.fx(), cm.fy(), smoothing);
    if (std::isnan(pLocal.x) || std::isnan(pLocal.y) || std::isnan(pLocal.z))
    {
        UWARN("Depth value not valid");
        return false;
    }
    rtabmap::Transform poseWorld = memory->getOptimizedPose(imageId);
    if (poseWorld.isNull())
    {
        UWARN("Image pose is Null");
        return false;
    }
    poseWorld = poseWorld * cm.localTransform();
    pWorld = rtabmap::util3d::transformPoint(pLocal, poseWorld);
    return true;
}

rtabmap::Transform Widget::localize(rtabmap::SensorData *sensorData)
{
    UASSERT(!sensorData->imageRaw().empty());

    rtabmap::Transform output;

    const rtabmap::CameraModel &cameraModel = sensorData->cameraModels()[0];

    // generate kpts
    if (memoryLoc.update(*sensorData))
    {
        UDEBUG("");
        const rtabmap::Signature *newS = memoryLoc.getLastWorkingSignature();
        UDEBUG("newWords=%d", (int)newS->getWords().size());
        std::map<int, float> likelihood;
        std::list<int> signaturesToCompare = uKeysList(memoryLoc.getSignatures());
        signaturesToCompare.remove(newS->id());
        UDEBUG("signaturesToCompare.size() = %d", signaturesToCompare.size());
        likelihood = memoryLoc.computeLikelihood(newS, signaturesToCompare);

        std::vector<int> topIds;
        likelihood.erase(-1);
        int topId;
        if (likelihood.size())
        {
            std::vector< std::pair<int, float> > top(TOP_K);
            std::partial_sort_copy(likelihood.begin(),
                                   likelihood.end(),
                                   top.begin(),
                                   top.end(),
                                   compareLikelihood);
            for (std::vector< std::pair<int, float> >::iterator it = top.begin(); it != top.end(); ++it)
            {
                topIds.push_back(it->first);
            }
            topId = topIds[0];
            UDEBUG("topId: %d", topId);
        }

        sensorData->setId(newS->id());

        // TODO: compare interatively until success
        output = memoryLoc.computeGlobalVisualTransform(topIds[0], sensorData->id());

        if (!output.isNull())
        {
            UDEBUG("global transform = %s", output.prettyPrint().c_str());
        }
        else
        {
            UWARN("transform is null, using pose of the closest image");
            //output = memoryLoc.getOptimizedPose(topId);
        }

        // remove new words from dictionary
        memoryLoc.deleteLocation(newS->id());
        memoryLoc.emptyTrash();
    }

    UINFO("output transform = %s", output.prettyPrint().c_str());

    return output;
}

bool Widget::compareLikelihood(std::pair<const int, float> const &l, std::pair<const int, float> const &r)
{
        return l.second > r.second;
}
