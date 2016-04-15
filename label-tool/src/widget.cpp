#include "widget.h"
#include "ui_widget.h"

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

    ui->slider->setRange(1, numImages); // image ID is 1-indexed

    return true;
}

void Widget::setSliderValue(int value)
{
    std::stringstream stream;
    stream << value << " out of " << numImages;

    ui->label_id->setText(QString::fromStdString(stream.str()));
}

void Widget::setLabel(const QString &name)
{
    ui->lineEdit->setText(name);
}

QString Widget::getLabel() const
{
    return ui->lineEdit->text();
}
