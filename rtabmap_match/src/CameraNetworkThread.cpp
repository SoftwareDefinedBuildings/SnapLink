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
    if(_camera)
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
            this->addData(networkEvent->getData());
            // TODO figure out where is event freed
        }
    }
}

void CameraNetworkThread::mainLoopKill()
{
    _dataAdded.release();
}

void CameraNetworkThread::mainLoop()
{
    std::vector<char> *data = NULL;
    size_t len;
    if (getData(data))
    {
        if (!_camera->addImage(data))
        {
            return;
        }
    }
    // take the image that was just added
    // the camera here basically wraps (void *) into SensorData 
    SensorData sensorData = _camera->takeImage();
    if(!sensorData.imageRaw().empty())
    {
        UINFO("New image %s", sensorData.filename.c_str());
        this->post(new CameraEvent(sensorData));
    }
}

void CameraNetworkThread::addData(std::vector<char> *data)
{
    bool notify = true;
    _dataMutex.lock();
    {
        _dataBuffer.push_back(data);

        while(_dataBufferMaxSize > 0 && _dataBuffer.size() > _dataBufferMaxSize)
        {
            UWARN("Data buffer is full, the oldest data is removed to add the new one.");
            _dataBuffer.pop_front();
            notify = false;
        }
    }
    _dataMutex.unlock();

    if(notify)
    {
        _dataAdded.release();
    }
}

bool CameraNetworkThread::getData(std::vector<char> *&data)
{
    bool dataFilled = false;
    _dataAdded.acquire();
    _dataMutex.lock();
    {
        if(!_dataBuffer.empty())
        {
            data = _dataBuffer.front();
            _dataBuffer.pop_front();
            dataFilled = true;
        }
    }
    _dataMutex.unlock();
    return dataFilled;
}

} // namespace rtabmap
