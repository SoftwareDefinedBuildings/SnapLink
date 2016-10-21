#include <iostream>
#include <QCoreApplication>
#include <libbw.h>
#include <allocations.h>
#include <sstream>
#include <string>
#include <bw_try.cpp>

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    BWServer bs; 
    return a.exec();
    
}
