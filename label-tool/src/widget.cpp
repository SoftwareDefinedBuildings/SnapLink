#include "widget.h"
#include "ui_widget.h"

Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);

    // setup DBDriver
    dbDriver = rtabmap::DBDriver::create();
}

Widget::~Widget()
{
    delete ui;
}

void Widget::setDbPath(char *name)
{
    dbPath = QString::fromUtf8(name);
    std::cout << "set db path to " << dbPath.toStdString() << std::endl;
}

bool Widget::openDatabase()
{
    if (!dbDriver->openConnection(dbPath.toStdString()))
    {
        return false;
    }

    return true;
}

void Widget::setLabel(const QString &name)
{
    ui->lineEdit->setText(name);
}

QString Widget::getLabel() const
{
    return ui->lineEdit->text();
}


void Widget::setSliderInterval(int interval)
{
    ui->horizontalSlider->setTickInterval(interval);
}
