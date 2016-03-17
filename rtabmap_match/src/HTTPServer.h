#pragma once

#include <QSemaphore>
#include <QObject>
#include <microhttpd.h>
#include "CameraNetwork.h"

#define POST_BUFFER_SIZE 300000
#define IMAGE_INIT_SIZE 300000

class CameraNetwork;

class HTTPServer :
    public QObject
{
public:
    HTTPServer(uint16_t port, unsigned int maxClients);
    virtual ~HTTPServer();

    bool start();
    void stop();

    void setMaxClients(unsigned int maxClients);
    unsigned int getNumClients();
    void setCamera(CameraNetwork *camera);

protected:
    virtual bool event(QEvent *event);

private:
    // TODO use camelCase
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
    static int send_page(struct MHD_Connection *connection, const std::string &page, int status_code);

private:
    uint16_t _port;
    struct MHD_Daemon *_daemon;
    unsigned int _maxClients;
    unsigned int _numClients;
    CameraNetwork *_camera;
};

enum ConnectionType
{
    GET = 0,
    POST = 1
};

typedef struct
{
    enum ConnectionType connectiontype;
    struct MHD_PostProcessor *postprocessor;
    std::vector<unsigned char> *data;
    std::string answerstring;
    int answercode;
    std::vector<std::string> names;
    QSemaphore detected;
} ConnectionInfo;
