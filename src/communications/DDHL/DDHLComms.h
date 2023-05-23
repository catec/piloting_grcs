#pragma once

#include <common/Types.h>
#include <core/IAction.h>
#include <dataModel/dtos/CommsLinkDDHLDTO.h>

#include <QObject>
#include <QTimer>
#include <memory>

#include "RestApiClient.h"
#include "gcs_communications_export.h"

namespace gcs {

class CommsLinkDDHLDTO;
class MissionResultDTO;
class AssetDTO;
class PILOTING_GRCS_COMMUNICATIONS_EXPORT DDHLComms : public QObject
{
    Q_OBJECT

  public:
    DDHLComms(QObject* parent = nullptr);
    ~DDHLComms();

  public Q_SLOTS:
    void downloadInspectionPlanFromDDHL(const CommsLinkDDHLDTO&, const QString&);
    void downloadSiteListFromDDHL(const CommsLinkDDHLDTO&, const SiteType&);
    void downloadAssetFromDDHL(const CommsLinkDDHLDTO&, const QString&);
    void uploadMissionResultListToDDHL(const CommsLinkDDHLDTO&, const QList<MissionResultDTO>&, const QString&);
    void abortServerOperation();

  private Q_SLOTS:
    void downloadedInspectionPlan(const QSharedPointer<InspectionPlanDTO>&);
    void downloadedSiteList(const QSharedPointer<SiteListDTO>&);
    void downloadedAssetFile(const QFile&);
    void updateAssetDownloadProgress(qint64, qint64);

    void receiveUploadedMissionACK(const QString&);
    void uploadMissionResultToDDHL(const MissionResultDTO&);

  private:
    void startProgressBar(const QString&);
    void finishProgressBar();
    void zipFolder(const QString&, QString&);
    void sendConsoleMsg(const MsgType&, const QString&);
    void sendPopupMsg(const QString&, const MsgType&, const QString&);

    std::unique_ptr<RestApiClient> _node;

    QList<quint16>          _missionsIndexesToUpload;
    QList<MissionResultDTO> _missionResultList;
    QString                 _missionResultsFolderPath;

    quint16 _currentIdCount;
    quint16 _maxIdListSize;

    QTimer _timeoutTimer;

    Parameters _uploadParams;

  Q_SIGNALS:
    void sendDTOToCore(ActionType actionType, QSharedPointer<IDTO> dto = nullptr);
    void uploadMissionToServer(const QString&);
    void abort();
};

} // namespace gcs
