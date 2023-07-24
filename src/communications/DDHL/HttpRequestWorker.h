#pragma once

#include <common/Types.h>

#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QObject>
#include <QString>

#include "HttpRequestInput.h"

namespace gcs {
class HttpRequestWorker : public QObject
{
    Q_OBJECT

  public:
    explicit HttpRequestWorker(QObject* parent = nullptr);

    void execute(HttpRequestInput*);

    QNetworkReply::NetworkError getErrorType() const;
    QByteArray                  getResponse() const;
    QString                     getErrorStr() const;

  public Q_SLOTS:
    void abort();

  private Q_SLOTS:
    void finish();
    void readyReadPackage();

  private:
    QString httpAttributeEncode(const QString&, const QString&);

    void generateRequestContent(HttpRequestInput& input, QByteArray& request_content, QString& boundary);
    void executeRequest(HttpRequestInput* input, QByteArray request_content, QString boundary);

    QNetworkAccessManager* _manager;
    QNetworkReply*         _reply;

    QNetworkReply::NetworkError _errorType;
    QByteArray                  _response;
    QString                     _errorStr;

  Q_SIGNALS:
    void executionFinished(HttpRequestWorker* worker);
    void sendConsoleMsg(const MsgType&, const QString&);
    void updateDownloadProgress(qint64 bytesRead, qint64 totalBytes);
};
} // namespace gcs
