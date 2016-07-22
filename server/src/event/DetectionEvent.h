#pragma once

#include <QEvent>
#include <vector>
#include "stage/HTTPServer.h"

class DetectionEvent :
    public QEvent
{
public:
    DetectionEvent(std::unique_ptr< std::vector<std::string> > &&names, const ConnectionInfo *conInfo);

    std::unique_ptr< std::vector<std::string> > getNames();
    const ConnectionInfo *conInfo() const;

    static QEvent::Type type();

private:
    static const QEvent::Type _type;
    std::unique_ptr< std::vector<std::string> > _names;
    const ConnectionInfo *_conInfo;
};
