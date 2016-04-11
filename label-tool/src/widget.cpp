#include "widget.h"
#include "ui_widget.h"

Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);
}

Widget::~Widget()
{
    delete ui;
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
