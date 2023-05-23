#include "DDHLComms.h"

#include <QsLog/QsLog.h>
#include <common/QuaZip/JlCompress.h>
#include <core/dtos/BasicStringDTO.h>
#include <dataModel/dtos/AssetDTO.h>
#include <dataModel/dtos/AssetProgressDTO.h>
#include <dataModel/dtos/CommsLinkDDHLDTO.h>
#include <dataModel/dtos/CommsProgressDTO.h>
#include <dataModel/dtos/InspectionPlanDTO.h>
#include <dataModel/dtos/MissionResultDTO.h>
#include <dataModel/dtos/MsgConsolePopupDTO.h>

#include <QCoreApplication>
#include <QFile>
#include <chrono>
#include <thread>

namespace gcs {

DDHLComms::DDHLComms(QObject* parent) :
        QObject(parent), _node(std::make_unique<RestApiClient>(this)), _timeoutTimer(this)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    // clang-format off
    connect(_node.get(), &RestApiClient::downloadedInspectionPlan,
            this,        &DDHLComms::downloadedInspectionPlan);
    
    connect(_node.get(), &RestApiClient::downloadedSiteList,
            this,        &DDHLComms::downloadedSiteList);

    connect(_node.get(), &RestApiClient::successfullyUploadedMissionACK,
            this,        &DDHLComms::receiveUploadedMissionACK);
 
    connect(_node.get(), &RestApiClient::downloadedAssetFile,
         this,           &DDHLComms::downloadedAssetFile);
 
    connect(_node.get(), &RestApiClient::sendPopupMsg,
            this,        &DDHLComms::sendPopupMsg);
 
    connect(_node.get(), &RestApiClient::finishProgressBar,
            this,        &DDHLComms::finishProgressBar);
 
    connect(_node.get(), &RestApiClient::updateDownloadProgress,
            this,        &DDHLComms::updateAssetDownloadProgress);
 
    connect(this,        &DDHLComms::abort,
            _node.get(), &RestApiClient::abort);
    // clang-format on

    connect(&_timeoutTimer, &QTimer::timeout, [=]() {
        sendPopupMsg(
                tr("Download data from DDHL"),
                MsgType::WARNING,
                tr("The connection to DDHL could not be stablished. Please, try it again."));
        finishProgressBar();
    });
    _timeoutTimer.setSingleShot(true);
    _timeoutTimer.setInterval(100000); /// 100 Hz
}

DDHLComms::~DDHLComms()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

void DDHLComms::downloadSiteListFromDDHL(const CommsLinkDDHLDTO& commsLinkDDHL, const SiteType& siteType)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    /// \note Parameters to DDHL server
    Parameters params;

    auto endpointType = commsLinkDDHL.getEndpointMode();
    if (endpointType == DDHLEndpointMode::IP) {
        params.url_str = "http://" + commsLinkDDHL.getTargetIP() + ":" + QString::number(commsLinkDDHL.getTargetPort())
                       + "/apis";
    } else {
        params.url_str = commsLinkDDHL.getTargetUrl();
    }

    params.id_str   = "Inspectionplan/site/" + QString::number(static_cast<uint8_t>(siteType));
    params.username = commsLinkDDHL.getUsername();
    params.password = commsLinkDDHL.getPassword();

    // /// \note Parameters to mock-up server
    // Parameters params;
    // params.url_str = "http://" + commsLinkDDHL.getTargetIP() + ":" +
    //                  QString::number(commsLinkDDHL.getTargetPort()) + "/apis";
    // params.id_str = "Inspectionplan/?UniqueID=" + inspectionPlanUUID;

    _node->getSiteListJSonResult(params);

    _timeoutTimer.start();
    startProgressBar("Downloading Inspection Plans Sites from DDHL");
}

void DDHLComms::downloadInspectionPlanFromDDHL(const CommsLinkDDHLDTO& commsLinkDDHL, const QString& inspectionPlanUUID)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    /// \note Parameters to DDHL server
    Parameters params;

    auto endpointType = commsLinkDDHL.getEndpointMode();
    if (endpointType == DDHLEndpointMode::IP) {
        params.url_str = "http://" + commsLinkDDHL.getTargetIP() + ":" + QString::number(commsLinkDDHL.getTargetPort())
                       + "/apis";
    } else {
        params.url_str = commsLinkDDHL.getTargetUrl();
    }

    params.id_str   = "Inspectionplan/" + inspectionPlanUUID + "/All/";
    params.username = commsLinkDDHL.getUsername();
    params.password = commsLinkDDHL.getPassword();

    // /// \note Parameters to mock-up server
    // Parameters params;
    // params.url_str = "http://" + commsLinkDDHL.getTargetIP() + ":" +
    //                  QString::number(commsLinkDDHL.getTargetPort()) + "/apis";
    // params.id_str = "Inspectionplan/?UniqueID=" + inspectionPlanUUID;

    _node->getInspectionPlanJSonResult(params);

    _timeoutTimer.start();
    startProgressBar("Downloading Inspection Plan from DDHL");
}

void DDHLComms::downloadAssetFromDDHL(const CommsLinkDDHLDTO& commsLinkDDHL, const QString& url)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    auto       nc_url   = url;
    auto       baseUrl  = QString("https://isense-piloting.iccs.gr/apis/File/");
    QString    fileUUID = nc_url.remove(baseUrl);
    Parameters params;

    auto endpointType = commsLinkDDHL.getEndpointMode();
    if (endpointType == DDHLEndpointMode::IP) {
        params.url_str = "http://" + commsLinkDDHL.getTargetIP() + ":" + QString::number(commsLinkDDHL.getTargetPort())
                       + "/apis";
    } else {
        params.url_str = commsLinkDDHL.getTargetUrl();
    }

    params.id_str   = "File/Download/" + fileUUID;
    params.username = commsLinkDDHL.getUsername();
    params.password = commsLinkDDHL.getPassword();

    _node->getPCDResult(params);

    _timeoutTimer.start();

    auto dtoToCore            = createSharedDTO<AssetProgressDTO>();
    dtoToCore->getBytesRead() = 0;

    Q_EMIT sendDTOToCore(ActionType::UPDATE_ASSET_PROGRESS, QSharedPointer<IDTO>(dtoToCore));
}

void DDHLComms::downloadedInspectionPlan(const QSharedPointer<InspectionPlanDTO>& receivedInspPlan)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _timeoutTimer.stop();
    finishProgressBar();

    sendConsoleMsg(MsgType::INFO, "Successfully downloaded Inspection Plan with UUID: " + receivedInspPlan->getUUID());

    Q_EMIT sendDTOToCore(ActionType::UPDATE_INSPECTION_PLAN, QSharedPointer<IDTO>(receivedInspPlan));
    Q_EMIT sendDTOToCore(ActionType::NEW_MISSION);
}

void DDHLComms::downloadedSiteList(const QSharedPointer<SiteListDTO>& receivedSiteList)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _timeoutTimer.stop();
    finishProgressBar();

    sendConsoleMsg(MsgType::INFO, "Successfully downloaded Site List");

    Q_EMIT sendDTOToCore(ActionType::UPDATE_SITE_LIST, QSharedPointer<IDTO>(receivedSiteList));
}

void DDHLComms::downloadedAssetFile(const QFile& assetFile)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _timeoutTimer.stop();
    finishProgressBar();

    QFileInfo fileInfo(assetFile.fileName());
    QString   absPath = fileInfo.absoluteFilePath();

    if (!assetFile.exists()) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << "Path to pointcloud does not exist:" << absPath;
        return;
    }

    QLOG_DEBUG() << __PRETTY_FUNCTION__ << "Reading asset from:" << absPath;

    auto basicStringDTO         = createSharedDTO<BasicStringDTO>();
    basicStringDTO->getString() = absPath;

    Q_EMIT sendDTOToCore(ActionType::UPDATE_ASSET_PATH, QSharedPointer<IDTO>(basicStringDTO));
}

void DDHLComms::uploadMissionResultListToDDHL(
        const CommsLinkDDHLDTO&        commsLinkDDHL,
        const QList<MissionResultDTO>& missionsToUpload,
        const QString&                 missionResultsFolderPath)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _missionResultList        = missionsToUpload;
    _maxIdListSize            = missionsToUpload.size();
    _currentIdCount           = 1;
    _missionResultsFolderPath = missionResultsFolderPath;

    auto endpointType = commsLinkDDHL.getEndpointMode();
    if (endpointType == DDHLEndpointMode::IP) {
        _uploadParams.url_str = "http://" + commsLinkDDHL.getTargetIP() + ":"
                              + QString::number(commsLinkDDHL.getTargetPort()) + "/apis";
    } else {
        _uploadParams.url_str = commsLinkDDHL.getTargetUrl();
    }

    _uploadParams.id_str   = "Mission/gRCS/";
    _uploadParams.username = commsLinkDDHL.getUsername();
    _uploadParams.password = commsLinkDDHL.getPassword();

    uploadMissionResultToDDHL(_missionResultList.first());
}

void DDHLComms::uploadMissionResultToDDHL(const MissionResultDTO& mission)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QLOG_DEBUG() << __PRETTY_FUNCTION__ << "Uploading mission result with name: " << mission.getName();

    try {
        QString    savedMissionZipPath = "";
        const auto missionResultFolderName
                = QString("mission_%1")
                          .arg(_missionResultList.first().getCreationTimeStamp().toString("yyyyMMddhhmmss"));
        zipFolder(missionResultFolderName, savedMissionZipPath);

        if (savedMissionZipPath.isEmpty()) {
            throw std::runtime_error("Error creating zip file");
        }

        _uploadParams.missionName = mission.getName();
        _node->postResult(_uploadParams, savedMissionZipPath);

        startProgressBar(QString("Uploading mission result (%1) to DDHL (%2/%3)")
                                 .arg(mission.getName())
                                 .arg(_currentIdCount)
                                 .arg(_maxIdListSize));

        sendConsoleMsg(MsgType::INFO, QString("Uploading mission result (%1) to DDHL.").arg(mission.getName()));
    } catch (const std::exception& e) {
        QLOG_ERROR() << __PRETTY_FUNCTION__ << "-" << e.what();
        sendConsoleMsg(
                MsgType::ERROR,
                QString("Cannot upload mission result (%1) to DDHL due to: %2").arg(_currentIdCount).arg(e.what()));
        finishProgressBar();
    }
}

void DDHLComms::receiveUploadedMissionACK(const QString& uploadedMissionName)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (uploadedMissionName == _missionResultList.first().getName()) {
        finishProgressBar();
        sendConsoleMsg(
                MsgType::INFO, QString("Correctly uploaded mission result (%1) to DDHL.").arg(uploadedMissionName));

        auto dtoToCore = createSharedDTO<MissionResultDTO>();
        *dtoToCore     = _missionResultList.first();
        Q_EMIT sendDTOToCore(ActionType::REMOVE_MISSION_RESULT, QSharedPointer<IDTO>(dtoToCore));

        _missionResultList.removeFirst();
        _currentIdCount++;
    } else {
        QLOG_WARN() << __PRETTY_FUNCTION__ << "Invalid index ACK received";
    }

    QLOG_DEBUG() << __PRETTY_FUNCTION__ << "Uploaded mission result with name: " << uploadedMissionName;

    if (_missionResultList.isEmpty()) {
        sendPopupMsg(tr("Mission Result Upload"), MsgType::INFO, tr("Correctly uploaded all missions to DDHL."));
    } else {
        uploadMissionResultToDDHL(_missionResultList.first());
    }
}

void DDHLComms::zipFolder(const QString& folderName, QString& savedFolderPath)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QDir parentFolderDir(_missionResultsFolderPath);
    if (!parentFolderDir.exists()) {
        const auto msg = QString("Mission results folder %1 does not exist").arg(parentFolderDir.absolutePath());
        throw std::runtime_error(msg.toStdString());
    }

    QDir folderDir(parentFolderDir.absoluteFilePath(folderName));
    if (!folderDir.exists()) {
        const auto msg = QString("Mission result folder %1 does not exist").arg(folderDir.absolutePath());
        throw std::runtime_error(msg.toStdString());
    }

    QString zipFileName = folderDir.absolutePath() + ".zip";
    if (!JlCompress::compressDir(zipFileName, folderDir.absolutePath())) {
        const auto msg
                = QString("Cannot compress folder %1 to zip file %2").arg(folderDir.absolutePath()).arg(zipFileName);
        throw std::runtime_error(msg.toStdString());
    }
    savedFolderPath = zipFileName;
}

void DDHLComms::updateAssetDownloadProgress(qint64 bytesRead, qint64 totalBytes)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QLOG_DEBUG() << __PRETTY_FUNCTION__ << "Bytes Read: " << bytesRead << " Total Bytes: " << totalBytes;
    _timeoutTimer.start();

    auto dtoToCore             = createSharedDTO<AssetProgressDTO>();
    dtoToCore->getBytesRead()  = bytesRead;
    dtoToCore->getTotalBytes() = totalBytes;

    Q_EMIT sendDTOToCore(ActionType::UPDATE_ASSET_PROGRESS, QSharedPointer<IDTO>(dtoToCore));
}

void DDHLComms::abortServerOperation()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    Q_EMIT abort();
}

void DDHLComms::startProgressBar(const QString& msg)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    auto dtoToCore                    = createSharedDTO<CommsProgressDTO>();
    dtoToCore->getCommsProgressType() = CommsProgressType::START;
    dtoToCore->getCommsProgressInfo() = msg;
    sendDTOToCore(ActionType::UPDATE_COMMS_PROGRESS, QSharedPointer<IDTO>(dtoToCore));
}

void DDHLComms::finishProgressBar()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    /// \note. Wait a bit, for quick transactions where there is no time to have launched the START
    auto dieTime = QTime::currentTime().addMSecs(100);
    while (QTime::currentTime() < dieTime) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
    }

    if (_timeoutTimer.isActive()) {
        _timeoutTimer.stop();
    }

    auto dtoToCore                    = createSharedDTO<CommsProgressDTO>();
    dtoToCore->getCommsProgressType() = CommsProgressType::STOP;
    sendDTOToCore(ActionType::UPDATE_COMMS_PROGRESS, QSharedPointer<IDTO>(dtoToCore));
}

void DDHLComms::sendConsoleMsg(const MsgType& type, const QString& msg)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    auto msgConsoleDTO          = createSharedDTO<MsgConsoleDTO>();
    msgConsoleDTO->getMsgType() = type;
    msgConsoleDTO->getMessage() = msg;
    Q_EMIT sendDTOToCore(ActionType::UPDATE_MSG_CONSOLE, QSharedPointer<IDTO>(msgConsoleDTO));
}

void DDHLComms::sendPopupMsg(const QString& title, const MsgType& type, const QString& msg)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    auto msgConsoleDTO          = createSharedDTO<MsgConsolePopupDTO>();
    msgConsoleDTO->getTitle()   = title;
    msgConsoleDTO->getMsgType() = type;
    msgConsoleDTO->getMessage() = msg;
    Q_EMIT sendDTOToCore(ActionType::UPDATE_MSG_CONSOLE, QSharedPointer<IDTO>(msgConsoleDTO));
}

} // namespace gcs
