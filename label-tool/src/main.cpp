#include <iostream>
#include <QApplication>
#include <QResource>
#include <rtabmap/utilite/UEventsManager.h>

#include "widget.h"

int main(int argc, char *argv[])
{
    ULogger::setType(ULogger::kTypeConsole);
    ULogger::setLevel(ULogger::kInfo);

    QResource::registerResource("label_dot.rcc");

    QApplication a(argc, argv);
    Widget w;

    if (argc < 2) {
        std::cout << "Need at least 1 argument" << std::endl;
        return -1;
    }

    // open database
    w.setDbPath(argv[1]);
    if (!w.openDatabase())
    {
        return -1;
    }
    if (!w.setSliderRange())
    {
        std::cout << "Database does not have any images" << std::endl;
        return -1;
    }

    w.setLabel("enter label name");

    w.show();

    return a.exec();
}
