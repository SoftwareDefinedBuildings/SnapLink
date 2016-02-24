#include <rtabmap/utilite/ULogger.h>
#include <strings.h>
#include <string.h>

#include "HTTPServer.h"
#include "NetworkEvent.h"
#include "DetectionEvent.h"

namespace rtabmap
{
const std::string busypage = "This server is busy, please try again later.";
const std::string completepage = "The upload has been completed.";
const std::string errorpage = "This doesn't seem to be right.";
const std::string servererrorpage = "An internal server error has occured.";


// ownership transferred
HTTPServer::HTTPServer(uint16_t port, unsigned int maxClients):
    _port(port),
    _maxClients(maxClients),
    _numClients(0),
    _daemon(NULL)
{
}

HTTPServer::~HTTPServer()
{
}

bool HTTPServer::start()
{
    // start MHD daemon, listening on port number _port
    _daemon = MHD_start_daemon(MHD_USE_SELECT_INTERNALLY, _port, NULL, NULL,
                               &answer_to_connection, this,
                               MHD_OPTION_NOTIFY_COMPLETED, &request_completed, this,
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
    }
}

void HTTPServer::handleEvent(UEvent *event)
{
    if (event->getClassName().compare("DetectionEvent") == 0)
    {
        DetectionEvent *detectionEvent = (DetectionEvent *) event;
        _names = detectionEvent->getNames();
        _Detected.release();
    }
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
    UDEBUG("");
    HTTPServer *httpServer = (HTTPServer *) cls;

    if (*con_cls == NULL)
    {
        ConnectionInfo *con_info;

        if (httpServer->_numClients >= httpServer->_maxClients)
        {
            return send_page(connection, busypage, MHD_HTTP_SERVICE_UNAVAILABLE);
        }

        con_info = new ConnectionInfo();
        if (con_info == NULL)
        {
            return MHD_NO;
        }

        // reserve enough space for an image
        con_info->data = new std::vector<unsigned char>();
        if (con_info->data == NULL)
        {
            delete con_info;
            return MHD_NO;
        }
        con_info->data->reserve(IMAGE_INIT_SIZE);

        if (strcasecmp(method, MHD_HTTP_METHOD_POST) == 0)
        {
            con_info->postprocessor = MHD_create_post_processor(connection, POST_BUFFER_SIZE, iterate_post, (void *)con_info);

            if (con_info->postprocessor == NULL)
            {
                delete con_info->data;
                delete con_info;
                return MHD_NO;
            }

            httpServer->_numClients++;

            con_info->connectiontype = POST;
            con_info->answercode = MHD_HTTP_OK;
            con_info->answerstring = completepage;
        }
        else
        {
            con_info->connectiontype = GET;
        }

        *con_cls = (void *) con_info;

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
            if (!con_info->data->empty())
            {
                httpServer->post(new NetworkEvent(con_info->data));
                con_info->data = NULL;
            }

            // wait for the result to come
            httpServer->_Detected.acquire();

            if (!httpServer->_names.empty())
            {
                con_info->answerstring = httpServer->_names.at(0);
            }
            else
            {
                con_info->answerstring = "";
            }
            httpServer->_names.clear();
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
    UDEBUG("");
    ConnectionInfo *con_info = (ConnectionInfo *) coninfo_cls;

    con_info->answerstring = servererrorpage;
    con_info->answercode = MHD_HTTP_INTERNAL_SERVER_ERROR;

    if (strcmp(key, "file") != 0)
    {
        return MHD_NO;
    }

    if (size > 0)
    {
        con_info->data->insert(con_info->data->end(), (unsigned char *) data, (unsigned char *) data + size);
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
        if (con_info->postprocessor != NULL)
        {
            MHD_destroy_post_processor(con_info->postprocessor);
            httpServer->_numClients--;
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

} // namespace rtabmap
