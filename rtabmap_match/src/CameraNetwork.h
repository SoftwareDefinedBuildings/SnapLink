#pragma once

#include "rtabmap/core/Camera.h"

#define PORT 8080
#define POSTBUFFERSIZE 512
#define MAXCLIENTS 2

class UTimer;

namespace rtabmap
{

class CameraNetwork :
    public Camera
{
public:
    CameraNetwork(uint16_t port = PORT,
            int maxClients = MAXCLIENTS,
            bool rectifyImages = false,
            bool isDepth = false,
            float imageRate = 0,
            const Transform & localTransform = Transform::getIdentity());
    virtual ~CameraNetwork();

    virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
    virtual bool isCalibrated() const;
    virtual std::string getSerial() const;

protected:
    virtual SensorData captureImage();

private:
    int CameraNetwork::answer_to_connection(void *cls,
                                            struct MHD_Connection *connection, 
                                            const char *url, 
                                            const char *method,
                                            const char *version, 
                                            const char *upload_data, 
                                            size_t *upload_data_size,
                                            void **con_cls);
    int iterate_post(void *coninfo_cls,
                     enum MHD_ValueKind kind,
                     const char *key,
                     const char *filename,
                     const char *content_type,
                     const char *transfer_encoding,
                     const char *data, 
                     uint64_t off,
                     size_t size)
    void request_completed(void *cls,
                           struct MHD_Connection *connection, 
                           void **con_cls,
                           enum MHD_RequestTerminationCode toe)
    int send_page(struct MHD_Connection *connection, const char* page, int status_code);

    uint16_t _port;
    unsigned int _maxClients;
    unsigned int _numClients;
    bool _rectifyImages;
    bool _isDepth;

    std::string _cameraName;
    CameraModel _model;
    struct MHD_Daemon *_daemon;
};

} // namespace rtabmap
