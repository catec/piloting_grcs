#include "HttpRequestWorker.h"

#include <common/Types.h>

#include <QBuffer>
#include <QDateTime>
#include <QEventLoop>
#include <QFileInfo>
#include <QUrl>
namespace gcs {

HttpRequestWorker::HttpRequestWorker(QObject* parent) : QObject(parent), _manager(NULL)
{
    qsrand(QDateTime::currentDateTime().toTime_t());

    _manager = new QNetworkAccessManager(this);
}

QNetworkReply::NetworkError HttpRequestWorker::getErrorType() const
{
    return _errorType;
}

QByteArray HttpRequestWorker::getResponse() const
{
    return _response;
}

QString HttpRequestWorker::getErrorStr() const
{
    return _errorStr;
}

QString HttpRequestWorker::httpAttributeEncode(const QString& attributeName, const QString& input)
{
    QString    result{""};
    const auto input_utf8 = QString::fromUtf8(input.toUtf8());

    const bool need_utf8_encoding
            = std::find_if(
                      input_utf8.begin(),
                      input_utf8.end(),
                      [](const QChar c) { return (c == '\\' || c == '/' || c == '\0' || c < ' ' || c > '~'); })
           != input_utf8.end();

    // If no special characters, return simple version
    if (!need_utf8_encoding) {
        result = QString("%1=\"%2\"").arg(attributeName, input_utf8);
    } else {
        // Encode the string with percent-encoding
        QByteArray input_bytes           = input_utf8.toUtf8();
        QString    input_percent_encoded = "";
        for (int i = 0; i < input_bytes.length(); i++) {
            unsigned char c = static_cast<unsigned char>(input_bytes.at(i));
            if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z')) {
                input_percent_encoded += c;
            } else {
                input_percent_encoded += "%" + QString::number(c, 16).toUpper();
            }
        }

        // Return enhanced version with UTF-8 support
        result = QString("%1=\"%2\"; %1*=utf-8''%3").arg(attributeName, input_utf8, input_percent_encoded);
    }

    return result;
}

void HttpRequestWorker::execute(HttpRequestInput* input)
{
    /// \note. Reset variables
    _response  = "";
    _errorType = QNetworkReply::NoError;
    _errorStr  = "";

    /// \note Decide on the variable layout
    input->var_layout = input->files.length() > 0 ? HttpRequestVarLayout::MULTIPART : HttpRequestVarLayout::NOT_SET;
    if (input->var_layout == HttpRequestVarLayout::NOT_SET) {
        input->var_layout = input->http_method == Literals::getMethod || input->http_method == Literals::headMethod
                                  ? HttpRequestVarLayout::ADDRESS
                                  : HttpRequestVarLayout::URL_ENCODED;
    }

    /// \note. Prepare request content
    QByteArray request_content = "";
    QString    boundary        = "";
    generateRequestContent(*input, request_content, boundary);

    /// \note. Execute desired request
    executeRequest(input, request_content, boundary);
}

void HttpRequestWorker::generateRequestContent(HttpRequestInput& input, QByteArray& request_content, QString& boundary)
{
    if (input.var_layout == HttpRequestVarLayout::ADDRESS || input.var_layout == HttpRequestVarLayout::URL_ENCODED) {
        /// \note.  If variable layout is ADDRESS or URL_ENCODED
        if (input.vars.count() > 0) {
            bool first = true;
            foreach (QString key, input.vars.keys()) {
                if (!first) {
                    request_content.append("&");
                }
                first = false;

                request_content.append(QUrl::toPercentEncoding(key));
                request_content.append("=");
                request_content.append(QUrl::toPercentEncoding(input.vars.value(key)));
            }

            if (input.var_layout == HttpRequestVarLayout::ADDRESS) {
                input.url_str   += "?" + request_content;
                request_content  = "";
            }
        }
    } else {
        /// \note.  If variable layout is MULTIPART
        boundary = "__-----------------------" + QString::number(QDateTime::currentDateTime().toTime_t())
                 + QString::number(qrand());
        QString boundary_delimiter = "--";
        QString new_line           = "\r\n";

        // add variables
        foreach (QString key, input.vars.keys()) {
            // add boundary
            request_content.append(boundary_delimiter);
            request_content.append(boundary);
            request_content.append(new_line);

            // add header
            request_content.append("Content-Disposition: form-data; ");
            request_content.append(httpAttributeEncode("name", key));
            request_content.append(new_line);
            request_content.append("Content-Type: text/plain");
            request_content.append(new_line);

            // add header to body splitter
            request_content.append(new_line);

            // add variable content
            request_content.append(input.vars.value(key));
            request_content.append(new_line);
        }

        // add files
        for (QList<HttpRequestInputFileElement>::iterator file_info = input.files.begin();
             file_info != input.files.end();
             file_info++) {
            QFileInfo fi(file_info->local_filename);

            // ensure necessary variables are available
            if (file_info->local_filename == NULL || file_info->local_filename.isEmpty()
                || file_info->variable_name == NULL || file_info->variable_name.isEmpty() || !fi.exists()
                || !fi.isFile() || !fi.isReadable()) {
                // silent abort for the current file
                continue;
            }

            QFile file(file_info->local_filename);
            if (!file.open(QIODevice::ReadOnly)) {
                // silent abort for the current file
                continue;
            }

            // ensure filename for the request
            if (file_info->request_filename == NULL || file_info->request_filename.isEmpty()) {
                file_info->request_filename = fi.fileName();
                if (file_info->request_filename.isEmpty()) {
                    file_info->request_filename = "file";
                }
            }

            // add boundary
            request_content.append(boundary_delimiter);
            request_content.append(boundary);
            request_content.append(new_line);

            // add header
            request_content.append(QString("Content-Disposition: form-data; %1; %2")
                                           .arg(httpAttributeEncode("name", file_info->variable_name),
                                                httpAttributeEncode("filename", file_info->request_filename)));
            request_content.append(new_line);

            if (file_info->mime_type != NULL && !file_info->mime_type.isEmpty()) {
                request_content.append("Content-Type: ");
                request_content.append(file_info->mime_type);
                request_content.append(new_line);
            }

            request_content.append("Content-Transfer-Encoding: binary");
            request_content.append(new_line);

            // add header to body splitter
            request_content.append(new_line);

            // add file content
            request_content.append(file.readAll());
            request_content.append(new_line);

            file.close();
        }

        // add end of body
        request_content.append(boundary_delimiter);
        request_content.append(boundary);
        request_content.append(boundary_delimiter);
    }
}

void HttpRequestWorker::executeRequest(HttpRequestInput* input, QByteArray request_content, QString boundary)
{
    QNetworkRequest request = QNetworkRequest(QUrl(input->url_str));
    request.setRawHeader("TE", "deflate,gzip;q=0.3");
    request.setRawHeader("Connection", "keep-alive");
    request.setRawHeader("User-Agent", "PILOTING gRCS");
    QString    concatenated = input->username + ":" + input->password;
    QByteArray data         = concatenated.toLocal8Bit().toBase64();
    QString    headerData   = "Basic " + data;
    request.setRawHeader("Authorization", headerData.toLocal8Bit());

    if (input->var_layout == HttpRequestVarLayout::URL_ENCODED) {
        request.setHeader(QNetworkRequest::ContentTypeHeader, "application/x-www-form-urlencoded");
    } else if (input->var_layout == HttpRequestVarLayout::MULTIPART) {
        request.setHeader(QNetworkRequest::ContentTypeHeader, "multipart/form-data; boundary=" + boundary);
    }

    if (input->http_method == Literals::getMethod) {
        _reply = _manager->get(request);
    } else if (input->http_method == Literals::postMethod) {
        _reply = _manager->post(request, request_content);
    } else if (input->http_method == Literals::putMethod) {
        _reply = _manager->put(request, request_content);
    } else if (input->http_method == Literals::headMethod) {
        _reply = _manager->head(request);
    } else if (input->http_method == Literals::deleteMethod) {
        _reply = _manager->deleteResource(request);
    } else {
        QBuffer buff(&request_content);
        _reply = _manager->sendCustomRequest(request, input->http_method.toLatin1(), &buff);
    }

    connect(_reply, &QNetworkReply::readyRead, this, &HttpRequestWorker::readyReadPackage);
    connect(_reply, &QNetworkReply::downloadProgress, this, &HttpRequestWorker::updateDownloadProgress);
    connect(_reply, &QNetworkReply::finished, this, &HttpRequestWorker::finish);
}

void HttpRequestWorker::readyReadPackage()
{
    auto error_code = _reply->error();
    if (error_code == QNetworkReply::NoError) {
        _response.append(_reply->readAll());
    } else {
        _errorStr = _reply->errorString();

        sendConsoleMsg(
                MsgType::WARNING,
                QString("Error trying to download HTTP package in GET operation with code: %1").arg(_errorStr));
        if (_reply) {
            _reply->deleteLater();
        }
        Q_EMIT executionFinished(this);
    }
}

void HttpRequestWorker::finish()
{
    _errorType = _reply->error();
    if (_errorType != QNetworkReply::NoError) {
        _errorStr = _reply->errorString() + " - " + ToString(_errorType);
    }

    if (_reply) {
        _reply->deleteLater();
    }

    Q_EMIT executionFinished(this);
}

void HttpRequestWorker::abort()
{
    if (_reply) {
        if (_reply->isRunning()) {
            _reply->abort();
        }
    }
}

} // namespace gcs