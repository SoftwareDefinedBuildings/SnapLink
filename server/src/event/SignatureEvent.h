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
    SignatureEvent(std::unique_ptr< std::vector<int> > &&wordIds, std::unique_ptr<rtabmap::SensorData> &&sensorData, std::unique_ptr< std::vector<Signature *> > &&signatures, ConnectionInfo *conInfo);

    std::unique_ptr< std::vector<int> > takeWordIds();
    std::unique_ptr<rtabmap::SensorData> takeSensorData();
    std::unique_ptr< std::vector<Signature *> > takeSignatures();
    ConnectionInfo *conInfo() const;

    static QEvent::Type type();

private:
    static const QEvent::Type _type;
    std::unique_ptr< std::vector<int> > _wordIds;
    std::unique_ptr<rtabmap::SensorData> _sensorData;
    std::unique_ptr< std::vector<Signature *> > _signatures;
    ConnectionInfo *_conInfo;
};
