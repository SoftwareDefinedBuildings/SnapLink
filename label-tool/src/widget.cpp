#include "widget.h"
#include "ui_widget.h"
#include <iostream>
#include <rtabmap/core/SensorData.h>

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

void Widget::setLabel(const QString &name)
{
    ui->lineEdit->setText(name);
}

QString Widget::getLabel() const
{
    return ui->lineEdit->text();
}
