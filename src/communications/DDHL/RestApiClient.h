#pragma once

#include <common/Types.h>

#include <QFile>
#include <QObject>
#include <memory>

#include "HttpRequestInput.h"
#include "HttpRequestWorker.h"
#include "JSon2DTO.h"

namespace gcs {

struct Parameters {
    QString url_str;
    QString id_str;
    QString username;
    QString password;
    QString missionName;
};

class RestApiClient : public QObject
{
    Q_OBJECT

  public:
    explicit RestApiClient(QObject* parent = nullptr);
    ~RestApiClient();

    void getInspectionPlanJSonResult(Parameters params);
    void getSiteListJSonResult(Parameters params);
    void postResult(Parameters params, const QString& filePathToPost);
    void getPCDResult(Parameters params);

  private Q_SLOTS:
    void handleGetInspectionPlanJSonResult(HttpRequestWorker* worker);
    void handleGetSiteListJSonResult(HttpRequestWorker* worker);
    void handlePostResult(HttpRequestWorker* worker);
    void handleGetPCDResult(HttpRequestWorker* worker);

  private:
    QString                            _lastMissionUploadedName;
    std::unique_ptr<HttpRequestWorker> _worker;
    JSon2DTO                           _json2DTO;

  Q_SIGNALS:
    void abort();
    void downloadedInspectionPlan(const QSharedPointer<InspectionPlanDTO>&);
    void downloadedSiteList(const QSharedPointer<SiteListDTO>&);
    void downloadedAssetFile(const QFile&);
    void successfullyUploadedMissionACK(QString);
    void updateDownloadProgress(qint64 bytesRead, qint64 totalBytes);
    void sendPopupMsg(const QString&, const MsgType&, const QString&);
    void sendConsoleMsg(const MsgType&, const QString&);
    void finishProgressBar();
};

} // namespace gcs
