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
    unsigned int flags = MHD_USE_SELECT_INTERNALLY | MHD_USE_EPOLL_LINUX_ONLY;
    _daemon = MHD_start_daemon(flags, port, nullptr, nullptr,
                               &answerConnection, static_cast<void *>(this),
                               MHD_OPTION_NOTIFY_COMPLETED, &requestCompleted, static_cast<void *>(this),
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

int HTTPServer::getMaxClients() const
{
    return _maxClients;
}

int HTTPServer::getNumClients() const
{
    return _numClients;
}

void HTTPServer::setNumClients(int numClients)
{
    _numClients = numClients;
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
        std::unique_ptr<SessionInfo> sessionInfo = detectionEvent->takeSessionInfo();
        sessionInfo->names = detectionEvent->takeNames();
        sessionInfo->detected->release();
        return true;
    }
    else if (event->type() == FailureEvent::type())
    {
        FailureEvent *failureEvent = static_cast<FailureEvent *>(event);
        std::unique_ptr<SessionInfo> sessionInfo = failureEvent->takeSessionInfo();
        sessionInfo->detected->release();
        return true;
    }
    return QObject::event(event);
}

int HTTPServer::answerConnection(void *cls,
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

        if (httpServer->getNumClients() >= httpServer->getMaxClients())
        {
            return sendPage(connection, busypage, MHD_HTTP_SERVICE_UNAVAILABLE);
        }

        std::unique_ptr<SessionInfo> sessionInfo(new SessionInfo());

        sessionInfo->timeInfo.overall_start = 0;
        sessionInfo->timeInfo.keypoints = 0;
        sessionInfo->timeInfo.descriptors = 0;
        sessionInfo->timeInfo.vwd = 0;
        sessionInfo->timeInfo.search = 0;
        sessionInfo->timeInfo.pnp = 0;

        // reserve enough space for an image
        sessionInfo->data.reserve(IMAGE_INIT_SIZE);
        
        sessionInfo->names.reset(new std::vector<std::string>());
        sessionInfo->detected.reset(new QSemaphore());

        if (strcasecmp(method, MHD_HTTP_METHOD_POST) == 0)
        {
            sessionInfo->postProcessor = MHD_create_post_processor(connection, POST_BUFFER_SIZE, iteratePost, (void *)&sessionInfo);

            if (sessionInfo->postProcessor == nullptr)
            {
                return MHD_NO;
            }

            httpServer->setNumClients(httpServer->getNumClients() + 1);

            sessionInfo->sessionType = POST;
            sessionInfo->answerCode.reset(new int(MHD_HTTP_OK));
            sessionInfo->answerString.reset(new std::string(completepage));
        }
        else
        {
            sessionInfo->sessionType = GET;
        }

        *con_cls = static_cast<void *>(&sessionInfo);

        return MHD_YES;
    }

    if (strcasecmp(method, MHD_HTTP_METHOD_GET) == 0)
    {
        // we do not accept GET request
        return sendPage(connection, errorpage, MHD_HTTP_SERVICE_UNAVAILABLE);
    }

    if (strcasecmp(method, MHD_HTTP_METHOD_POST) == 0)
    {
        std::unique_ptr<SessionInfo> sessionInfo = std::move(*static_cast<std::unique_ptr<SessionInfo> *>(*con_cls));

        if (*upload_data_size != 0)
        {
            MHD_post_process(sessionInfo->postProcessor, upload_data, *upload_data_size);
            *upload_data_size = 0;

            return MHD_YES;
        }
        else
        {
            std::shared_ptr<QSemaphore> detected = sessionInfo->detected;

            if (!sessionInfo->data.empty())
            {
                // all data are received
                sessionInfo->timeInfo.overall_start = getTime(); // log start of processing
                // seperate ownership of data from sessionInfo
                QCoreApplication::postEvent(httpServer->_camera, new NetworkEvent(sessionInfo.release()));
            }

            // wait for the result to come
            detected->acquire();

            std::shared_ptr< std::vector<std::string> > names = sessionInfo->names;
            std::shared_ptr<std::string> answerString = sessionInfo->answerString;
            std::shared_ptr<int> answerCode = sessionInfo->answerCode;

            if (names != nullptr && !names->empty())
            {
                *answerString = names->at(0);
            }
            else
            {
                *answerString = "None";
            }
            return sendPage(connection, *answerString, *answerCode);
        }
    }

    return sendPage(connection, errorpage, MHD_HTTP_BAD_REQUEST);
}

int HTTPServer::iteratePost(void *coninfo_cls,
                             enum MHD_ValueKind kind,
                             const char *key,
                             const char *filename,
                             const char *content_type,
                             const char *transfer_encoding,
                             const char *data,
                             uint64_t off,
                             size_t size)
{
    SessionInfo *con_info = (SessionInfo *) coninfo_cls;

    *(con_info->answerString) = servererrorpage;
    *(con_info->answerCode) = MHD_HTTP_INTERNAL_SERVER_ERROR;

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
            con_info->cameraInfo.fx = atof(buf);
        }
        else if (strcmp(key, "fy") == 0)
        {
            char buf[size + 1];
            memcpy(buf, data, size);
            buf[size] = 0;
            con_info->cameraInfo.fy = atof(buf);
        }
        else if (strcmp(key, "cx") == 0)
        {
            char buf[size + 1];
            memcpy(buf, data, size);
            buf[size] = 0;
            con_info->cameraInfo.cx = atof(buf);
        }
        else if (strcmp(key, "cy") == 0)
        {
            char buf[size + 1];
            memcpy(buf, data, size);
            buf[size] = 0;
            con_info->cameraInfo.cy = atof(buf);
        }
    }

    *(con_info->answerString) = completepage;
    *(con_info->answerCode) = MHD_HTTP_OK;

    return MHD_YES;
}

void HTTPServer::requestCompleted(void *cls,
                                   struct MHD_Connection *connection,
                                   void **con_cls,
                                   enum MHD_RequestTerminationCode toe)
{
    UDEBUG("");
    HTTPServer *httpServer = (HTTPServer *) cls;
    SessionInfo *con_info = (SessionInfo *) *con_cls;

    if (con_info == nullptr)
    {
        return;
    }

    if (con_info->sessionType == POST)
    {
        con_info->timeInfo.overall = getTime() - con_info->timeInfo.overall_start; // log processing end time

        UINFO("TAG_TIME overall %ld", con_info->timeInfo.overall);
        UINFO("TAG_TIME keypoints %ld", con_info->timeInfo.keypoints);
        UINFO("TAG_TIME descriptors %ld", con_info->timeInfo.descriptors);
        UINFO("TAG_TIME vwd %ld", con_info->timeInfo.vwd);
        UINFO("TAG_TIME search %ld", con_info->timeInfo.search);
        UINFO("TAG_TIME pnp %ld", con_info->timeInfo.pnp);

        if (con_info->postProcessor != nullptr)
        {
            MHD_destroy_post_processor(con_info->postProcessor);
            httpServer->setNumClients(httpServer->getNumClients() - 1);
        }
    }

    delete con_info;
    *con_cls = nullptr;
}

int HTTPServer::sendPage(struct MHD_Connection *connection, const std::string &page, int status_code)
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
