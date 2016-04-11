#include <iostream>
#include <QApplication>

#include "widget.h"

#include "rtabmap/gui/DatabaseViewer.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Widget w;

    if (argc < 2) {
        std::cout << "Need at least 1 argument" << std::endl;
        return -1;
    }

    //w.setLabel("enter label name");
    w.setLabel(argv[1]);

    w.setSliderInterval(5);

    w.show();

    return a.exec();
}
