#ifndef WIDGET_H
#define WIDGET_H

#include <iostream>
#include <QWidget>
#include "rtabmap/core/DBDriver.h"

namespace Ui {
class Widget;
}

class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = 0);
    ~Widget();

    void setDbPath(char *);
    bool openDatabase(void);
    bool setSliderRange(void);

    void setLabel(const QString &name);
    QString getLabel() const;

private slots:
    void setSliderValue(int);

private:
    Ui::Widget *ui;

    rtabmap::DBDriver *dbDriver;
    QString dbPath;
    int numImages;
};

#endif // WIDGET_H
