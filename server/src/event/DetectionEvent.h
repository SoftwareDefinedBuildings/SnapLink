#pragma once

#include <QEvent>
#include <vector>
#include <memory>
#include "data/PerfData.h"

class DetectionEvent :
    public QEvent
{
public:
    DetectionEvent(std::unique_ptr< std::vector<std::string> > &&names, std::unique_ptr<PerfData> &&perfData, const void *session = nullptr);

    std::unique_ptr< std::vector<std::string> > takeNames();
    std::unique_ptr<PerfData> takePerfData();
    const void *getSession();

    static QEvent::Type type();

private:
    static const QEvent::Type _type;
    const void *_session;
    std::unique_ptr< std::vector<std::string> > _names;
    std::unique_ptr<PerfData> _perfData;
};
