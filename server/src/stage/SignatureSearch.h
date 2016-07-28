#pragma once

#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Parameters.h>
#include <memory>
#include <QObject>
#include <QEvent>
#include "data/Signatures.h"
#include "stage/Perspective.h"

#define TOP_K 1

class Perspective;

class SignatureSearch :
    public QObject
{
public:
    SignatureSearch();
    virtual ~SignatureSearch();

    void setSignatures(std::unique_ptr<Signatures> &&signatures);
    void setPerspective(Perspective *perspective);

protected:
    virtual bool event(QEvent *event);

private:
    std::vector<Signature *> searchSignatures(const std::vector<int> &wordIds) const;

private:
    std::unique_ptr<Signatures> _signatures;
    Perspective *_perspective;
};
