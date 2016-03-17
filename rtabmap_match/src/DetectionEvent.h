#pragma once

#include <QEvent>
#include <vector>
#include "HTTPServer.h"

class DetectionEvent :
    public QEvent
{
public:
    // ownership transfer
    DetectionEvent(const std::vector<std::string> *names, const ConnectionInfo *conInfo);

    // get the names of the detected objects
    const std::vector<std::string> *names() const;
    const ConnectionInfo *conInfo() const;

    static QEvent::Type type();

private:
    static QEvent::Type _type;
    const std::vector<std::string> *_names;
    const ConnectionInfo *_conInfo;
};
