#include <rtabmap/core/Camera.h>
#include <rtabmap/core/CameraEvent.h>
#include <rtabmap/utilite/ULogger.h>
#include "CameraThreadStream.h"

namespace rtabmap
{

// ownership transferred
CameraThreadStream::CameraThreadStream(Camera * camera) :
_camera(camera)
{
    UASSERT(_camera != NULL);
}

CameraThreadStream::~CameraThreadStream()
{
    join(true);
    if(_camera)
    {
        delete _camera;
    }
}

void CameraThreadStream::setImageRate(float imageRate)
{
    if(_camera)
    {
        _camera->setImageRate(imageRate);
    }
}

void CameraThreadStream::mainLoop()
{
    SensorData data = _camera->takeImage();
    if(!data.imageRaw().empty())
    {
        this->post(new CameraEvent(data));
    }
}

} // namespace rtabmap
