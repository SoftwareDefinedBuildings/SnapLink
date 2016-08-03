#pragma once

#include <rtabmap/core/SensorData.h>
#include <QEvent>
#include "data/PerfData.h"
#include "data/Signature.h"

class SignatureEvent :
    public QEvent
{
public:
    // ownership transfer
    SignatureEvent(std::unique_ptr< std::vector<int> > &&wordIds, std::unique_ptr<rtabmap::SensorData> &&sensorData, std::vector< std::unique_ptr<Signature> > &&signatures, std::unique_ptr<PerfData> &&perfData, const void *session);

    std::unique_ptr< std::vector<int> > takeWordIds();
    std::unique_ptr<rtabmap::SensorData> takeSensorData();
    std::vector< std::unique_ptr<Signature> > takeSignatures();
    std::unique_ptr<PerfData> takePerfData();
    const void *getSession();

    static QEvent::Type type();

private:
    static const QEvent::Type _type;
    const void *_session;
    std::unique_ptr< std::vector<int> > _wordIds;
    std::unique_ptr<rtabmap::SensorData> _sensorData;
    std::vector< std::unique_ptr<Signature> > _signatures;
    std::unique_ptr<PerfData> _perfData;
};
