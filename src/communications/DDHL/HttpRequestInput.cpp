
#include "HttpRequestInput.h"

namespace gcs {

HttpRequestInput::HttpRequestInput()
{
    initialize();
}

HttpRequestInput::HttpRequestInput(const QString& v_url_str, const QString& v_http_method)
{
    initialize();
    url_str     = v_url_str;
    http_method = v_http_method;
}

void HttpRequestInput::initialize()
{
    var_layout  = NOT_SET;
    url_str     = "";
    http_method = "GET";
    username    = "";
    password    = "";
}

void HttpRequestInput::add_var(const QString& key, const QString& value)
{
    vars[key] = value;
}

void HttpRequestInput::add_file(
        const QString& variable_name,
        const QString& local_filename,
        const QString& request_filename,
        const QString& mime_type)
{
    HttpRequestInputFileElement file;
    file.variable_name    = variable_name;
    file.local_filename   = local_filename;
    file.request_filename = request_filename;
    file.mime_type        = mime_type;
    files.append(file);
}

void HttpRequestInput::add_auth(const QString& user, const QString& passWord)
{
    username = user;
    password = passWord;
}
} // namespace gcs
