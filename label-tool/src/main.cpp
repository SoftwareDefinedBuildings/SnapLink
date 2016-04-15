#include <iostream>
#include <QApplication>

#include "widget.h"

int main(int argc, char *argv[])
{
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
        std::cout << "Couldn't open database " << argv[1] << std::endl;
        return -1;
    }

    //w.setLabel("enter label name");
    w.setLabel(argv[1]);

    w.setSliderInterval(5);

    w.show();

    return a.exec();
}
