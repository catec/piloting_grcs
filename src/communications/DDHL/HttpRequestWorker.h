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

    QNetworkReply::NetworkError error_type;
    QByteArray                  response;
    QString                     error_str;
    QByteArray                  request_content{""};

  public Q_SLOTS:
    void abort();

  private Q_SLOTS:
    void finish();
    void readyReadPackage();

  private:
    QString httpAttributeEncode(const QString&, const QString&);
    void    resetVariables();
    void    setVarLayout(HttpRequestInput*);
    void    prepareRequestConnection(HttpRequestInput*);
    void    prepareConnection(HttpRequestInput*);
    void    prepareUrlEncodedContent(HttpRequestInput*);
    void    prepareMultipartContent(HttpRequestInput*);
    void    prepareRequestContent(HttpRequestInput*);
    void    prepareMultipartVar(const QString& key, const QString& value);

    QNetworkAccessManager* _manager;
    QNetworkReply*         _reply;

    const QString _boundary = "__-----------------------" + QString::number(QDateTime::currentDateTime().toTime_t())
                            + QString::number(qrand());
    const QString _boundary_delimiter = "--";
    const QString _new_line           = "\r\n";

  Q_SIGNALS:
    void executionFinished(HttpRequestWorker* worker);
    void sendConsoleMsg(const MsgType&, const QString&);
    void updateDownloadProgress(qint64 bytesRead, qint64 totalBytes);
};
} // namespace gcs
