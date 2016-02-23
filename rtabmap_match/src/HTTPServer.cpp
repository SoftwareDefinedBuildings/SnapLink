#include <rtabmap/utilite/ULogger.h>
#include <strings.h>
#include <string.h>
#include "HTTPServer.h"
#include "DetectionEvent.h"

namespace rtabmap
{

const char *askpage = "<html><body>\n\
                       Upload a file, please!<br>\n\
                       There are %u clients uploading at the moment.<br>\n\
                       <form action=\"/filepost\" method=\"post\" enctype=\"multipart/form-data\">\n\
                       <input name=\"file\" type=\"file\">\n\
                       <input type=\"submit\" value=\" Send \"></form>\n\
                       </body></html>";

const char *busypage =
  "<html><body>This server is busy, please try again later.</body></html>";

const char *completepage =
  "<html><body>The upload has been completed.</body></html>";

const char *errorpage =
  "<html><body>This doesn't seem to be right.</body></html>";
const char *servererrorpage =
  "<html><body>An internal server error has occured.</body></html>";
const char *fileexistspage =
  "<html><body>This file already exists.</body></html>";


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
    if(event->getClassName().compare("DetectionEvent") == 0)
    {
        DetectionEvent *detectionEvent = (DetectionEvent *) event;
        // TODO respond to http post
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
    HTTPServer *camera = (HTTPServer *) cls;

    if (*con_cls == NULL)
    {
        ConnectionInfo *con_info;

        if (camera->_numClients >= camera->_maxClients)
        { 
            return send_page(connection, busypage, MHD_HTTP_SERVICE_UNAVAILABLE);
        }

        con_info = new ConnectionInfo();
        if (con_info == NULL)
        {
            return MHD_NO;
        }

        con_info->fp = NULL;

        if (strcasecmp(method, MHD_HTTP_METHOD_POST) == 0) 
        {            
            con_info->postprocessor = MHD_create_post_processor(connection, POSTBUFFERSIZE, iterate_post, (void *)con_info);   

            if (con_info->postprocessor == NULL) 
            {
                delete con_info; 
                return MHD_NO;
            }
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
            if (con_info->fp != NULL)
            {
                fclose(con_info->fp);
                con_info->fp = NULL;
            }
            // Now it is safe to open and inspect the file before calling send_page with a response
            // TODO send event and wait on CV
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
    FILE *fp;
  
    con_info->answerstring = servererrorpage;
    con_info->answercode = MHD_HTTP_INTERNAL_SERVER_ERROR;
  
    if (strcmp(key, "file") != 0)
    {
        return MHD_NO;
    }

    if (!con_info->fp)
    {
        if ((fp = fopen(filename, "rb")) != NULL)
        {
            fclose(fp);
            con_info->answerstring = fileexistspage;
            con_info->answercode = MHD_HTTP_FORBIDDEN;
            return MHD_NO;
        }
  
        con_info->fp = fopen(filename, "ab");
        if (!con_info->fp)
        {
            return MHD_NO;
        }
    }
  
    if (size > 0)
    {
        if (!fwrite(data, sizeof(char), size, con_info->fp))
        {
            return MHD_NO;
        }
    }
  
    con_info->answerstring = completepage;
    con_info->answercode = MHD_HTTP_OK;
  
    UINFO("MHD_YES %s %s", key, filename, content_type);
    return MHD_YES;
}

void HTTPServer::request_completed(void *cls,
                                   struct MHD_Connection *connection, 
                                   void **con_cls,
                                   enum MHD_RequestTerminationCode toe)
{
    UDEBUG("");
    HTTPServer *camera = (HTTPServer *) cls;
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
            camera->_numClients--;
        }
  
        if (con_info->fp)
        {
            fclose (con_info->fp);
        }
    }
  
    delete con_info;
    *con_cls = NULL;
}

int HTTPServer::send_page(struct MHD_Connection *connection, const char* page, int status_code)
{
    int ret;
    struct MHD_Response *response;
  
    response = MHD_create_response_from_buffer (strlen (page), (void*) page, MHD_RESPMEM_MUST_COPY);
    if (!response) return MHD_NO;
 
    ret = MHD_queue_response (connection, status_code, response);
    MHD_destroy_response (response);

    return ret;
}


} // namespace rtabmap
