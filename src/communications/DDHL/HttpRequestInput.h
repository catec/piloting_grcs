#pragma once

#include <QNetworkReply>
#include <QObject>
#include <QString>

inline const char* ToString(const QNetworkReply::NetworkError v)
{
    switch (v) {
        case QNetworkReply::ConnectionRefusedError: {
            return "ConnectionRefusedError";
        }
        case QNetworkReply::RemoteHostClosedError: {
            return "RemoteHostClosedError";
        }
        case QNetworkReply::HostNotFoundError: {
            return "HostNotFoundError";
        }
        case QNetworkReply::TimeoutError: {
            return "TimeoutError";
        }
        case QNetworkReply::OperationCanceledError: {
            return "OperationCanceledError";
        }
        case QNetworkReply::SslHandshakeFailedError: {
            return "SslHandshakeFailedError";
        }
        case QNetworkReply::TemporaryNetworkFailureError: {
            return "TemporaryNetworkFailureError";
        }
        case QNetworkReply::NetworkSessionFailedError: {
            return "NetworkSessionFailedError";
        }
        case QNetworkReply::BackgroundRequestNotAllowedError: {
            return "BackgroundRequestNotAllowedError";
        }
        case QNetworkReply::TooManyRedirectsError: {
            return "TooManyRedirectsError";
        }
        case QNetworkReply::InsecureRedirectError: {
            return "InsecureRedirectError";
        }
        case QNetworkReply::ProxyConnectionRefusedError: {
            return "ProxyConnectionRefusedError";
        }
        case QNetworkReply::ProxyConnectionClosedError: {
            return "ProxyConnectionClosedError";
        }
        case QNetworkReply::ProxyNotFoundError: {
            return "ProxyNotFoundError";
        }
        case QNetworkReply::ProxyTimeoutError: {
            return "ProxyTimeoutError";
        }
        case QNetworkReply::ProxyAuthenticationRequiredError: {
            return "ProxyAuthenticationRequiredError";
        }
        case QNetworkReply::ContentAccessDenied: {
            return "ContentAccessDenied";
        }
        case QNetworkReply::ContentOperationNotPermittedError: {
            return "ContentOperationNotPermittedError";
        }
        case QNetworkReply::ContentNotFoundError: {
            return "ContentNotFoundError";
        }
        case QNetworkReply::AuthenticationRequiredError: {
            return "AuthenticationRequiredError";
        }
        case QNetworkReply::ContentReSendError: {
            return "ContentReSendError";
        }
        case QNetworkReply::ContentConflictError: {
            return "ContentConflictError";
        }
        case QNetworkReply::ContentGoneError: {
            return "ContentGoneError";
        }
        case QNetworkReply::InternalServerError: {
            return "InternalServerError";
        }
        case QNetworkReply::OperationNotImplementedError: {
            return "OperationNotImplementedError";
        }
        case QNetworkReply::ServiceUnavailableError: {
            return "ServiceUnavailableError";
        }
        case QNetworkReply::ProtocolUnknownError: {
            return "ProtocolUnknownError";
        }
        case QNetworkReply::ProtocolInvalidOperationError: {
            return "ProtocolInvalidOperationError";
        }
        case QNetworkReply::UnknownNetworkError: {
            return "UnknownNetworkError";
        }
        case QNetworkReply::UnknownProxyError: {
            return "UnknownProxyError";
        }
        case QNetworkReply::UnknownContentError: {
            return "UnknownContentError";
        }
        case QNetworkReply::ProtocolFailure: {
            return "ProtocolFailure";
        }
        case QNetworkReply::UnknownServerError: {
            return "UnknownServerError";
        }
        default: {
            return "UNKNOWN";
        }
    }
}

enum HttpRequestVarLayout { NOT_SET, ADDRESS, URL_ENCODED, MULTIPART };

struct HttpRequestInputFileElement {
    QString variable_name;
    QString local_filename;
    QString request_filename;
    QString mime_type;
};

namespace Literals {
static const QString getMethod    = QStringLiteral("GET");
static const QString postMethod   = QStringLiteral("POST");
static const QString putMethod    = QStringLiteral("PUT");
static const QString headMethod   = QStringLiteral("HEAD");
static const QString deleteMethod = QStringLiteral("DELETE");
} // namespace Literals

namespace gcs {

class HttpRequestInput
{
  public:
    HttpRequestInput();
    HttpRequestInput(const QString& v_url_str, const QString& v_http_method);

    void initialize();
    void add_var(const QString& key, const QString& value);
    void add_auth(const QString& user, const QString& passWord);
    void add_file(
            const QString& variable_name,
            const QString& local_filename,
            const QString& request_filename,
            const QString& mime_type);

    QString url_str;
    QString http_method;
    QString username;
    QString password;

    HttpRequestVarLayout               var_layout;
    QMap<QString, QString>             vars;
    QList<HttpRequestInputFileElement> files;
};
} // namespace gcs