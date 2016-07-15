#pragma once

#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Parameters.h>
#include <QObject>
#include <QEvent>
#include "RTABMapDBAdapter.h"
#include "Perspective.h"
#include "HTTPServer.h"
#include "Time.h"

#define TOP_K 1

class Perspective;
class HTTPServer;

class SignatureSearch :
    public QObject
{
public:
    SignatureSearch();
    virtual ~SignatureSearch();

    void setMemory(RTABMapDBAdapter *memory);
    void setPerspective(Perspective *vis);

protected:
    virtual bool event(QEvent *event);

private:
    std::vector<Signature *> searchSignatures(std::vector<int> wordIds, const rtabmap::SensorData *sensorData, void *context);

private:
    RTABMapDBAdapter *_memory;
    Perspective *_perspective;
};
