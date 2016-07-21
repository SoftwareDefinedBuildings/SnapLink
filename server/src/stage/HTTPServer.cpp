#include <rtabmap/utilite/ULogger.h>
#include <strings.h>
#include <string.h>
#include <cstdlib>
#include <QCoreApplication>

#include "stage/HTTPServer.h"
#include "event/NetworkEvent.h"
#include "event/DetectionEvent.h"
#include "event/FailureEvent.h"

#include "util/Time.h"

const std::string HTTPServer::busypage = "This server is busy, please try again later.";
const std::string HTTPServer::completepage = "The upload has been completed.";
const std::string HTTPServer::errorpage = "This doesn't seem to be right.";
const std::string HTTPServer::servererrorpage = "An internal server error has occured.";

// ownership transferred
HTTPServer::HTTPServer():
    _daemon(nullptr),
    _numClients(0),
    _camera(nullptr)
{
}

HTTPServer::~HTTPServer()
{
    stop();
    _numClients = 0;
    _camera = nullptr;
}

bool HTTPServer::start(uint16_t port, unsigned int maxClients)
{
    _maxClients = maxClients;

    // start MHD daemon, listening on port
    unsigned int flags = MHD_USE_THREAD_PER_CONNECTION | MHD_USE_POLL;
    _daemon = MHD_start_daemon(flags, port, nullptr, nullptr,
                               &answer_to_connection, static_cast<void *>(this),
                               MHD_OPTION_NOTIFY_COMPLETED, &request_completed, static_cast<void *>(this),
                               MHD_OPTION_END);
    if (_daemon == nullptr)
    {
        return false;
    }

    return true;
}

void HTTPServer::stop()
{
    if (_daemon != nullptr)
    {
        MHD_stop_daemon(_daemon);
        _daemon = nullptr;
    }
}

const unsigned int &HTTPServer::maxClients() const
{
    return _maxClients;
}

unsigned int &HTTPServer::numClients()
{
    return _numClients;
}

void HTTPServer::setCamera(CameraNetwork *camera)
{
    _camera = camera;
}

bool HTTPServer::event(QEvent *event)
{
    if (event->type() == DetectionEvent::type())
    {
        DetectionEvent *detectionEvent = static_cast<DetectionEvent *>(event);
        ConnectionInfo *conInfo = const_cast<ConnectionInfo *>(detectionEvent->conInfo());
        conInfo->names = detectionEvent->names();
        conInfo->detected.release();
        return true;
    }
    else if (event->type() == FailureEvent::type())
    {
        FailureEvent *failureEvent = static_cast<FailureEvent *>(event);
        ConnectionInfo *conInfo = const_cast<ConnectionInfo *>(failureEvent->conInfo());
        conInfo->names = nullptr;
        conInfo->detected.release();
        return true;
    }
    return QObject::event(event);
}

int HTTPServer::answer_to_connection(void *cls,
                                     struct MHD_Connection *connection,
                                     const char *url,
                                     const char *method,
                                     const char *version,
                                     const char *upload_data,
                                     size_t *upload_data_size,
                                     void **con_cls)
{
    HTTPServer *httpServer = static_cast<HTTPServer *>(cls);

    if (*con_cls == nullptr)
    {

        if (httpServer->numClients() >= httpServer->maxClients())
        {
            return send_page(connection, busypage, MHD_HTTP_SERVICE_UNAVAILABLE);
        }

        auto con_info = new ConnectionInfo();

        con_info->time.overall_start = 0;
        con_info->time.keypoints = 0;
        con_info->time.descriptors = 0;
        con_info->time.vwd = 0;
        con_info->time.search = 0;
        con_info->time.pnp = 0;

        // reserve enough space for an image
        con_info->data.reserve(IMAGE_INIT_SIZE);

        if (strcasecmp(method, MHD_HTTP_METHOD_POST) == 0)
        {
            con_info->postprocessor = MHD_create_post_processor(connection, POST_BUFFER_SIZE, iterate_post, (void *)con_info);

            if (con_info->postprocessor == nullptr)
            {
                delete con_info;
                return MHD_NO;
            }

            httpServer->numClients()++;

            con_info->names = nullptr;
            con_info->connectiontype = POST;
            con_info->answercode = MHD_HTTP_OK;
            con_info->answerstring = completepage;
        }
        else
        {
            con_info->connectiontype = GET;
        }

        *con_cls = static_cast<void *>(con_info);

        return MHD_YES;
    }

    if (strcasecmp(method, MHD_HTTP_METHOD_GET) == 0)
    {
        // we do not accept GET request
        return send_page(connection, errorpage, MHD_HTTP_SERVICE_UNAVAILABLE);
    }

    if (strcasecmp(method, MHD_HTTP_METHOD_POST) == 0)
    {
        ConnectionInfo *con_info = (ConnectionInfo *) *con_cls;

        if (*upload_data_size != 0)
        {
            MHD_post_process(con_info->postprocessor, upload_data, *upload_data_size);
            *upload_data_size = 0;

            return MHD_YES;
        }
        else
        {
            if (!con_info->data.empty())
            {
                // all data are received
                con_info->time.overall_start = getTime(); // log start of processing
                // seperate ownership of data from con_info
                QCoreApplication::postEvent(httpServer->_camera, new NetworkEvent(con_info));
            }

            // wait for the result to come
            con_info->detected.acquire();

            if (con_info->names != nullptr && !con_info->names->empty())
            {
                con_info->answerstring = con_info->names->at(0);
            }
            else
            {
                con_info->answerstring = "None";
            }
            delete con_info->names;
            con_info->names = nullptr;
            return send_page(connection, con_info->answerstring, con_info->answercode);
        }
    }

    return send_page(connection, errorpage, MHD_HTTP_BAD_REQUEST);
}

int HTTPServer::iterate_post(void *coninfo_cls,
                             enum MHD_ValueKind kind,
                             const char *key,
                             const char *filename,
                             const char *content_type,
                             const char *transfer_encoding,
                             const char *data,
                             uint64_t off,
                             size_t size)
{
    ConnectionInfo *con_info = (ConnectionInfo *) coninfo_cls;

    con_info->answerstring = servererrorpage;
    con_info->answercode = MHD_HTTP_INTERNAL_SERVER_ERROR;

    if (strcmp(key, "file") != 0 && strcmp(key, "fx") != 0 && strcmp(key, "fy") != 0 && strcmp(key, "cx") != 0 && strcmp(key, "cy") != 0)
    {
        return MHD_NO;
    }

    if (size > 0)
    {
        if (strcmp(key, "file") == 0)
        {
            con_info->data.insert(con_info->data.end(), data, data + size);
        }
        else if (strcmp(key, "fx") == 0)
        {
            char buf[size + 1];
            memcpy(buf, data, size);
            buf[size] = 0;
            con_info->fx = atof(buf);
        }
        else if (strcmp(key, "fy") == 0)
        {
            char buf[size + 1];
            memcpy(buf, data, size);
            buf[size] = 0;
            con_info->fy = atof(buf);
        }
        else if (strcmp(key, "cx") == 0)
        {
            char buf[size + 1];
            memcpy(buf, data, size);
            buf[size] = 0;
            con_info->cx = atof(buf);
        }
        else if (strcmp(key, "cy") == 0)
        {
            char buf[size + 1];
            memcpy(buf, data, size);
            buf[size] = 0;
            con_info->cy = atof(buf);
        }
    }

    con_info->answerstring = completepage;
    con_info->answercode = MHD_HTTP_OK;

    return MHD_YES;
}

void HTTPServer::request_completed(void *cls,
                                   struct MHD_Connection *connection,
                                   void **con_cls,
                                   enum MHD_RequestTerminationCode toe)
{
    UDEBUG("");
    HTTPServer *httpServer = (HTTPServer *) cls;
    ConnectionInfo *con_info = (ConnectionInfo *) *con_cls;

    if (con_info == nullptr)
    {
        return;
    }

    if (con_info->connectiontype == POST)
    {
        con_info->time.overall = getTime() - con_info->time.overall_start; // log processing end time

        UINFO("TAG_TIME overall %ld", con_info->time.overall);
        UINFO("TAG_TIME keypoints %ld", con_info->time.keypoints);
        UINFO("TAG_TIME descriptors %ld", con_info->time.descriptors);
        UINFO("TAG_TIME vwd %ld", con_info->time.vwd);
        UINFO("TAG_TIME search %ld", con_info->time.search);
        UINFO("TAG_TIME pnp %ld", con_info->time.pnp);

        if (con_info->postprocessor != nullptr)
        {
            MHD_destroy_post_processor(con_info->postprocessor);
            httpServer->numClients()--;
        }
    }

    delete con_info;
    *con_cls = nullptr;
}

int HTTPServer::send_page(struct MHD_Connection *connection, const std::string &page, int status_code)
{
    int ret;
    struct MHD_Response *response;

    response = MHD_create_response_from_buffer(page.length(), (void *) page.c_str(), MHD_RESPMEM_MUST_COPY);
    if (!response)
    {
        return MHD_NO;
    }

    ret = MHD_queue_response(connection, status_code, response);
    MHD_destroy_response(response);

    return ret;
}
