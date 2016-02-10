#pragma once

#include "rtabmap/core/Camera.h"
#include <microhttpd.h>

#define PORT 8080
#define POSTBUFFERSIZE 51200
#define MAXCLIENTS 2

class UTimer;

namespace rtabmap
{

class CameraNetwork :
    public Camera
{
public:
    CameraNetwork(uint16_t port = PORT,
            unsigned int maxClients = MAXCLIENTS,
            bool rectifyImages = false,
            bool isDepth = false,
            float imageRate = 0,
            const Transform & localTransform = Transform::getIdentity());
    virtual ~CameraNetwork();

    virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
    virtual bool isCalibrated() const;
    virtual std::string getSerial() const;
    
    // TODO use getter and setter
    unsigned int _maxClients;
    unsigned int _numClients;

protected:
    virtual SensorData captureImage();

private:
    static int answer_to_connection(void *cls,
                                    struct MHD_Connection *connection, 
                                    const char *url, 
                                    const char *method,
                                    const char *version, 
                                    const char *upload_data, 
                                    size_t *upload_data_size,
                                    void **con_cls);
    static int iterate_post(void *coninfo_cls,
                            enum MHD_ValueKind kind,
                            const char *key,
                            const char *filename,
                            const char *content_type,
                            const char *transfer_encoding,
                            const char *data, 
                            uint64_t off,
                            size_t size);
    static void request_completed(void *cls,
                                  struct MHD_Connection *connection, 
                                  void **con_cls,
                                  enum MHD_RequestTerminationCode toe);
    static int send_page(struct MHD_Connection *connection, const char* page, int status_code);

    uint16_t _port;
    bool _rectifyImages;
    bool _isDepth;

    std::string _cameraName;
    CameraModel _model;
    struct MHD_Daemon *_daemon;
};

} // namespace rtabmap
