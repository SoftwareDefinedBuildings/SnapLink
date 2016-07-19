#pragma once

#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Parameters.h>
#include <QObject>
#include <QEvent>
#include "data/Signatures.h"
#include "stage/Perspective.h"
#include "util/Time.h"

#define TOP_K 1

class Perspective;

class SignatureSearch :
    public QObject
{
public:
    SignatureSearch();
    virtual ~SignatureSearch();

    void setSignatures(Signatures *signatures);
    void setPerspective(Perspective *vis);

protected:
    virtual bool event(QEvent *event);

private:
    std::vector<Signature *> searchSignatures(std::vector<int> wordIds, const rtabmap::SensorData *sensorData, void *context);

private:
    Signatures *_signatures;
    Perspective *_perspective;
};
