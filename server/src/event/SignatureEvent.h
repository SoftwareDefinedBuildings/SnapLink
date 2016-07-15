#pragma once

#include <rtabmap/core/SensorData.h>
#include <QEvent>
#include "stage/HTTPServer.h"
#include "data/Signature.h"

class SignatureEvent :
    public QEvent
{
public:
    // ownership transfer
    SignatureEvent(std::vector<int> wordIds, rtabmap::SensorData *sensorData, std::vector<Signature *> signatures, ConnectionInfo *conInfo);

    std::vector<int> wordIds() const;
    rtabmap::SensorData *sensorData() const;
    std::vector<Signature *> signatures() const;
    ConnectionInfo *conInfo() const;

    static QEvent::Type type();

private:
    static const QEvent::Type _type;
    const std::vector<int> _wordIds;
    rtabmap::SensorData *_sensorData;
    const std::vector<Signature *> _signatures;
    ConnectionInfo *_conInfo;
};
