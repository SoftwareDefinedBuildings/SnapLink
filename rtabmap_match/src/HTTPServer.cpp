#include <rtabmap/utilite/ULogger.h>
#include <strings.h>
#include <string.h>
#include <cstdlib>
#include <QCoreApplication>

#include "HTTPServer.h"
#include "NetworkEvent.h"
#include "DetectionEvent.h"
#include "FailureEvent.h"

#include "Time.h"

const std::string HTTPServer::busypage = "This server is busy, please try again later.";
const std::string HTTPServer::completepage = "The upload has been completed.";
const std::string HTTPServer::errorpage = "This doesn't seem to be right.";
const std::string HTTPServer::servererrorpage = "An internal server error has occured.";

// ownership transferred
HTTPServer::HTTPServer():
    _daemon(NULL),
    _numClients(0),
    _camera(NULL)
{
}

HTTPServer::~HTTPServer()
{
    stop();
    _numClients = 0;
    _camera = NULL;
}

bool HTTPServer::start(uint16_t port, unsigned int maxClients)
{
    _maxClients = maxClients;

    // start MHD daemon, listening on port
    unsigned int flags = MHD_USE_THREAD_PER_CONNECTION | MHD_USE_POLL;
    _daemon = MHD_start_daemon(flags, port, NULL, NULL,
                               &answer_to_connection, static_cast<void *>(this),
                               MHD_OPTION_NOTIFY_COMPLETED, &request_completed, static_cast<void *>(this),
                               MHD_OPTION_END);
    if (_daemon == NULL)
    {
        return false;
    }

    return true;
}

void HTTPServer::stop()
{
    if (_daemon != NULL)
    {
        MHD_stop_daemon(_daemon);
        _daemon = NULL;
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
        conInfo->names = NULL;
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

    if (*con_cls == NULL)
    {

        if (httpServer->numClients() >= httpServer->maxClients())
        {
            return send_page(connection, busypage, MHD_HTTP_SERVICE_UNAVAILABLE);
        }

        ConnectionInfo *con_info = new ConnectionInfo();

        con_info->time_start = getTime(); // log start of processing

        // reserve enough space for an image
        con_info->data.reserve(IMAGE_INIT_SIZE);

        if (strcasecmp(method, MHD_HTTP_METHOD_POST) == 0)
        {
            con_info->postprocessor = MHD_create_post_processor(connection, POST_BUFFER_SIZE, iterate_post, (void *)con_info);

            if (con_info->postprocessor == NULL)
            {
                delete con_info;
                return MHD_NO;
            }

            httpServer->numClients()++;

            con_info->names = NULL;
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
                // seperate ownership of data from con_info
                QCoreApplication::postEvent(httpServer->_camera, new NetworkEvent(con_info));
            }

            // wait for the result to come
            int n = 1;
            int time = 5000; // time to wait (ms)
            bool acquired = con_info->detected.tryAcquire(n, time);

            if (acquired && con_info->names != NULL && !con_info->names->empty())
            {
                con_info->answerstring = con_info->names->at(0);
            }
            else
            {
                con_info->answerstring = "None";
            }
            delete con_info->names;
            con_info->names = NULL;
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

    if (strcmp(key, "file") != 0 && strcmp(key, "width") != 0 && strcmp(key, "height") != 0 && strcmp(key, "fx") != 0 && strcmp(key, "fy") != 0 && strcmp(key, "cx") != 0 && strcmp(key, "cy") != 0)
    {
        return MHD_NO;
    }

    if (size > 0)
    {
        if (strcmp(key, "file") == 0)
        {
            con_info->data.insert(con_info->data.end(), (unsigned char *) data, (unsigned char *) data + size);
        }
        else if (strcmp(key, "width") == 0)
        {
            char buf[size + 1];
            memcpy(buf, data, size);
            buf[size] = 0;
            con_info->width = atoi(buf);
        }
        else if (strcmp(key, "height") == 0)
        {
            char buf[size + 1];
            memcpy(buf, data, size);
            buf[size] = 0;
            con_info->height = atoi(buf);
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

    if (con_info == NULL)
    {
        return;
    }

    if (con_info->connectiontype == POST)
    {
        con_info->time_end = getTime(); // log processing end time

        // output timing data to log
        long time_overall = con_info->time_end - con_info->time_start;
        long time_surf = con_info->time_surf_end - con_info->time_surf_start;
        long time_closest = con_info->time_closest_end - con_info->time_closest_start;
        long time_pnp = con_info->time_pnp_end - con_info->time_pnp_start;
        UINFO("TAG_TIME overall %ld", time_overall);
        UINFO("TAG_TIME surf %ld", time_surf);
        UINFO("TAG_TIME closest_match %ld", time_closest);
        UINFO("TAG_TIME pnp %ld", time_pnp);

        if (con_info->postprocessor != NULL)
        {
            MHD_destroy_post_processor(con_info->postprocessor);
            httpServer->numClients()--;
        }
    }

    delete con_info;
    *con_cls = NULL;
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
