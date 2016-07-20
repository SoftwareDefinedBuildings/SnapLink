#pragma once

#include <QSemaphore>
#include <QObject>
#include <microhttpd.h>
#include "stage/CameraNetwork.h"

#define PORT 8080
#define MAX_CLIENTS 10
#define POST_BUFFER_SIZE 300000
#define IMAGE_INIT_SIZE 300000

class CameraNetwork;

class HTTPServer :
    public QObject
{
public:
    HTTPServer();
    virtual ~HTTPServer();

    bool start(uint16_t port = PORT, unsigned int maxClients = MAX_CLIENTS);
    void stop();

    const unsigned int &maxClients() const;
    unsigned int &numClients();
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
    static const std::string busypage;
    static const std::string completepage;
    static const std::string errorpage;
    static const std::string servererrorpage;

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
    long overall;
    long overall_start;
    long keypoints;
    long keypoints_start;
    long descriptors;
    long descriptors_start;
    long vwd;
    long vwd_start;
    long search;
    long search_start;
    long pnp;
    long pnp_start;
} TimeInfo;

typedef struct
{
    enum ConnectionType connectiontype;
    struct MHD_PostProcessor *postprocessor;
    std::vector<char> data;
    int width;
    int height;
    double fx;
    double fy;
    double cx;
    double cy;
    std::string answerstring;
    int answercode;
    const std::vector<std::string> *names;
    QSemaphore detected;
    TimeInfo time;
} ConnectionInfo;
