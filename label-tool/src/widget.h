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

    void setLabel(const QString &name);
    QString getLabel() const;

    void setSliderInterval(int);

private:
    Ui::Widget *ui;

    rtabmap::DBDriver *dbDriver;
    QString dbPath;
};

#endif // WIDGET_H
