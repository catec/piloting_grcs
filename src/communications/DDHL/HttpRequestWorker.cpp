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
    resetVariables();
    setVarLayout(input);
    prepareRequestContent(input);
    prepareConnection(input);
}

void HttpRequestWorker::resetVariables()
{
    response   = "";
    error_str  = "";
    error_type = QNetworkReply::NoError;
}

void HttpRequestWorker::setVarLayout(HttpRequestInput* input)
{
    input->var_layout = input->files.length() > 0 ? HttpRequestVarLayout::MULTIPART : HttpRequestVarLayout::NOT_SET;
    if (input->var_layout == HttpRequestVarLayout::NOT_SET) {
        input->var_layout = input->http_method == Literals::getMethod || input->http_method == Literals::headMethod
                                  ? HttpRequestVarLayout::ADDRESS
                                  : HttpRequestVarLayout::URL_ENCODED;
    }
}

void HttpRequestWorker::prepareRequestContent(HttpRequestInput* input)
{
    if (input->var_layout == HttpRequestVarLayout::ADDRESS || input->var_layout == HttpRequestVarLayout::URL_ENCODED) {
        prepareUrlEncodedContent(input);
    } else {
        prepareMultipartContent(input);
    }
}

void HttpRequestWorker::prepareUrlEncodedContent(HttpRequestInput* input)
{
    if (input->vars.isEmpty()) {
        return;
    }

    QStringList encodedVars;
    foreach (const QString& key, input->vars.keys()) {
        encodedVars.append(QUrl::toPercentEncoding(key) + "=" + QUrl::toPercentEncoding(input->vars.value(key)));
    }

    request_content = encodedVars.join("&").toUtf8();

    if (input->var_layout == HttpRequestVarLayout::ADDRESS) {
        input->url_str  += "?" + request_content;
        request_content  = "";
    }
}

void HttpRequestWorker::prepareMultipartContent(HttpRequestInput* input)
{
    foreach (const QString& key, input->vars.keys()) {
        prepareMultipartVar(key, input->vars.value(key));
    }

    foreach (const auto& file_info, input->files) {
        if (file_info.local_filename.isNull() || file_info.local_filename.isEmpty() || file_info.variable_name.isNull()
            || file_info.variable_name.isEmpty()) {
            continue;
        }

        QFile file(file_info.local_filename);
        if (!file.open(QIODevice::ReadOnly)) {
            continue;
        }

        const QString& requestFilename = file_info.request_filename.isNull() || file_info.request_filename.isEmpty()
                                               ? QFileInfo(file_info.local_filename).fileName()
                                               : file_info.request_filename;

        request_content += _boundary_delimiter + _boundary + _new_line;
        request_content += "Content-Disposition: form-data; " + httpAttributeEncode("name", file_info.variable_name)
                         + "; " + httpAttributeEncode("filename", requestFilename) + _new_line;
        if (!file_info.mime_type.isNull() && !file_info.mime_type.isEmpty()) {
            request_content += "Content-Type: " + file_info.mime_type + _new_line;
        }
        request_content += "Content-Transfer-Encoding: binary" + _new_line + _new_line;
        request_content += file.readAll() + _new_line;

        file.close();
    }

    request_content += _boundary_delimiter + _boundary + _boundary_delimiter;
}

void HttpRequestWorker::prepareMultipartVar(const QString& key, const QString& value)
{
    request_content += _boundary_delimiter + _boundary + _new_line;
    request_content += "Content-Disposition: form-data; name=\"" + key + "\"" + _new_line;
    request_content += _new_line;
    request_content += value + _new_line;
}

void HttpRequestWorker::prepareConnection(HttpRequestInput* input)
{
    QNetworkRequest request = QNetworkRequest(QUrl(input->url_str));
    // request.setTransferTimeout(100000);
    request.setRawHeader("TE", "deflate,gzip;q=0.3");
    request.setRawHeader("Connection", "TE, close");
    request.setRawHeader("User-Agent", "Agent name goes here");
    // HTTP Basic authentication header value: base64(username:password)
    QString    concatenated = input->username + ":" + input->password;
    QByteArray data         = concatenated.toLocal8Bit().toBase64();
    QString    headerData   = "Basic " + data;
    request.setRawHeader("Authorization", headerData.toLocal8Bit());

    if (input->var_layout == HttpRequestVarLayout::URL_ENCODED) {
        request.setHeader(QNetworkRequest::ContentTypeHeader, "application/x-www-form-urlencoded");
    } else if (input->var_layout == HttpRequestVarLayout::MULTIPART) {
        request.setHeader(QNetworkRequest::ContentTypeHeader, "multipart/form-data; boundary=" + _boundary);
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
        response.append(_reply->readAll());
    } else {
        error_str = _reply->errorString();

        sendConsoleMsg(
                MsgType::WARNING,
                QString("Error trying to download HTTP package in GET operation with code: %1").arg(error_str));
        if (_reply) {
            _reply->deleteLater();
        }
        Q_EMIT executionFinished(this);
    }
}

void HttpRequestWorker::finish()
{
    error_type = _reply->error();
    if (error_type != QNetworkReply::NoError) {
        error_str = _reply->errorString() + " - " + ToString(error_type);
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