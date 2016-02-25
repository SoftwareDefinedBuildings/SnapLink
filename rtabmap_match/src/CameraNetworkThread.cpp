#include <rtabmap/core/Camera.h>
#include <rtabmap/core/CameraEvent.h>
#include <rtabmap/utilite/ULogger.h>

#include "CameraNetworkThread.h"
#include "NetworkEvent.h"

namespace rtabmap
{

// ownership transferred
CameraNetworkThread::CameraNetworkThread(CameraNetwork *camera, unsigned int dataBufferMaxSize) :
    _camera(camera),
    _dataBufferMaxSize(dataBufferMaxSize)
{
    UASSERT(_camera != NULL);
}

CameraNetworkThread::~CameraNetworkThread()
{
    this->unregisterFromEventsManager();
    this->join(true);
    if (_camera)
    {
        delete _camera;
    }
}

void CameraNetworkThread::handleEvent(UEvent *event)
{
    if (this->isRunning())
    {
        if (event->getClassName().compare("NetworkEvent") == 0)
        {
            NetworkEvent *networkEvent = (NetworkEvent *) event;
            this->addData(networkEvent->payload(), networkEvent->context());
        }
    }
}

void CameraNetworkThread::mainLoopKill()
{
    _dataAdded.release();
}

void CameraNetworkThread::mainLoop()
{
    std::vector<unsigned char> *data = NULL;
    void *context = NULL;
    size_t len;
    if (getData(data, context))
    {
        if (!_camera->addImage(data))
        {
            return;
        }
    }
    // take the image that was just added
    SensorData sensorData = _camera->takeImage();
    if (!sensorData.imageRaw().empty())
    {
        this->post(new ImageEvent(sensorData, "", context));
    }
}

void CameraNetworkThread::addData(std::vector<unsigned char> *data, void *context)
{
    bool notify = true;
    _dataMutex.lock();
    {
        _dataBuffer.push_back(data);
        _contextBuffer.push_back(context);

        while (_dataBufferMaxSize > 0 && _dataBuffer.size() > _dataBufferMaxSize)
        {
            UWARN("Data buffer is full, the oldest data is removed to add the new one.");
            _dataBuffer.pop_front();
            _contextBuffer.pop_front();
            notify = false;
        }
    }
    _dataMutex.unlock();

    if (notify)
    {
        _dataAdded.release();
    }
}

bool CameraNetworkThread::getData(std::vector<unsigned char> *&data, void *&context)
{
    bool dataFilled = false;
    _dataAdded.acquire();
    _dataMutex.lock();
    {
        if (!_dataBuffer.empty() && _dataBuffer.size() == _contextBuffer.size())
        {
            data = _dataBuffer.front();
            context = _contextBuffer.front();
            _dataBuffer.pop_front();
            _contextBuffer.pop_front();
            dataFilled = true;
        }
    }
    _dataMutex.unlock();
    return dataFilled;
}

} // namespace rtabmap
