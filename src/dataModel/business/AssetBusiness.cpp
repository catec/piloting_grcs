#include "AssetBusiness.h"

#include <QsLog/QsLog.h>
#include <config.h>

#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>

#include "../dtos/AssetDTO.h"

namespace gcs {

AssetBusiness::AssetBusiness(AssetDTO& assetDTO) : _assetDTO(assetDTO)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

AssetBusiness::~AssetBusiness()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

QString AssetBusiness::obtainAssetsFolderPath()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QDir assetDir(QDir::home().absoluteFilePath(
            QString(".pilotingGrcs/v%1.%2/assets/").arg(PROJECT_VERSION_MAJOR).arg(PROJECT_VERSION_MINOR)));

    return assetDir.absoluteFilePath(QString("asset_%1").arg(_assetDTO.getUUID()));
}

bool AssetBusiness::checkIfNeedDownloadAsset()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    const auto currentAssetPath = obtainAssetsFolderPath();

    QDir currentAssetDir(currentAssetPath);

    /// \note If no folder with the asset id exists, it should be downloaded
    if (!currentAssetDir.exists()) {
        return true;
    }

    QFile assetInfoFile(currentAssetDir.absoluteFilePath("asset_info.json"));

    /// \note If the information file does not exist, it should be downloaded.
    if (!assetInfoFile.exists()) {
        return true;
    }

    /// \note If it is more recent than the one we have downloaded, it should be updated.
    auto downloadedUpdateTime = _assetDTO.getLastUpdateDateTime();
    auto savedUpdateTime      = obtainLastUpdateTimeFromFile(assetInfoFile.fileName());
    if (savedUpdateTime < downloadedUpdateTime) {
        return true;
    }

    return false;
}

QDateTime AssetBusiness::obtainLastUpdateTimeFromFile(const QString& assetInfoPath)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QFile assetInfoFile(assetInfoPath);
    if (!assetInfoFile.open(QIODevice::ReadOnly)) {
        const auto msg = QString("Could not read asset information file from: %1").arg(assetInfoFile.fileName());
        QLOG_ERROR() << __PRETTY_FUNCTION__ << " - " << msg;
        throw std::runtime_error(msg.toStdString());
    }

    auto       assetJDoc(QJsonDocument::fromJson(assetInfoFile.readAll()));
    const auto assetJO = assetJDoc.object();
    if (assetJO.value("UpdateDatetime") == QJsonValue::Undefined) {
        const auto msg = QString("UpdateDatetime field does not exist");
        QLOG_ERROR() << __PRETTY_FUNCTION__ << " - " << msg;
        throw std::runtime_error(msg.toStdString());
    }
    assetInfoFile.close();

    return QDateTime::fromString(assetJO["UpdateDatetime"].toString(), Qt::ISODateWithMs);
}

AssetDTO AssetBusiness::convertInformationJsonToAssetDTO()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    auto  currAssetPath = obtainAssetsFolderPath();
    QDir  currAssetDir(currAssetPath);
    QFile assetInfoFile(currAssetDir.absoluteFilePath("asset_info.json"));
    if (!assetInfoFile.open(QIODevice::ReadOnly)) {
        const auto msg = QString("Could not read asset information file from: %1").arg(assetInfoFile.fileName());
        QLOG_ERROR() << __PRETTY_FUNCTION__ << " - " << msg;
        throw std::runtime_error(msg.toStdString());
    }

    auto       assetJDoc(QJsonDocument::fromJson(assetInfoFile.readAll()));
    const auto assetJO = assetJDoc.object();

    /// \todo Check that all fields are valids and exists
    AssetDTO assetDTO;
    assetDTO.getUUID()               = assetJO["Uuid"].toString();
    assetDTO.getName()               = assetJO["Name"].toString();
    assetDTO.getCreationDateTime()   = QDateTime::fromString(assetJO["CreationDatetime"].toString(), Qt::ISODateWithMs);
    assetDTO.getLastUpdateDateTime() = QDateTime::fromString(assetJO["UpdateDatetime"].toString(), Qt::ISODateWithMs);

    FileDTO fileDTO;

    const auto filesInFolder = currAssetDir.entryInfoList(QDir::NoDot | QDir::NoDotDot | QDir::Files);
    for (const auto& file : filesInFolder) {
        if (file.suffix() == "pcd" || file.suffix() == "stl" || file.suffix() == "ply") {
            fileDTO.getPath() = file.absoluteFilePath();
        }
    }

    if (fileDTO.getPath().isEmpty()) {
        throw std::runtime_error(
                QString("Unable to read PCD file from asset folder: %1").arg(currAssetDir.path()).toStdString());
    }

    assetInfoFile.close();
    assetDTO.getLoadedFile() = fileDTO;

    return assetDTO;
}

void AssetBusiness::saveAssetInFolder()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QDir mainAssetsDir(QDir::home().absoluteFilePath(
            QString(".pilotingGrcs/v%1.%2/assets/").arg(PROJECT_VERSION_MAJOR).arg(PROJECT_VERSION_MINOR)));

    if (!QDir().mkpath(mainAssetsDir.path())) {
        const auto msg = QString("Main assets directory has not been created: %1").arg(mainAssetsDir.path());
        throw std::runtime_error(msg.toStdString());
    }

    QDir currentAssetDir(mainAssetsDir.absoluteFilePath(QString("asset_%1").arg(_assetDTO.getUUID())));
    if (!QDir().mkpath(currentAssetDir.path())) {
        const auto msg = QString("Asset directory has not been created: %1").arg(currentAssetDir.path());
        throw std::runtime_error(msg.toStdString());
    }

    QFileInfo assetFileInfo(_assetDTO.getLoadedFile().getPath());
    auto      originalFilename = getAssetFileName();

    if (originalFilename.isEmpty()) {
        originalFilename = assetFileInfo.fileName();
    }

    const auto assetFilePath = currentAssetDir.absoluteFilePath(originalFilename);

    QLOG_DEBUG() << __PRETTY_FUNCTION__ << "Copying asset from: " << _assetDTO.getLoadedFile().getPath()
                 << " to: " << assetFilePath;

    QFile::copy(_assetDTO.getLoadedFile().getPath(), assetFilePath);

    /// \note Update asset path to new location
    _assetDTO.getLoadedFile().getPath() = assetFilePath;

    convertAssetInfoToJson(currentAssetDir.absoluteFilePath("asset_info.json"));
}

void AssetBusiness::convertAssetInfoToJson(const QString& assetInfoPath)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QFile assetInfoFile(assetInfoPath);
    if (!assetInfoFile.open(QIODevice::WriteOnly)) {
        const auto msg = QString("Could not create file to save asset information: %1").arg(assetInfoFile.fileName());
        QLOG_ERROR() << __PRETTY_FUNCTION__ << " - " << msg;
        throw std::runtime_error(msg.toStdString());
    }

    QJsonObject assetJO;
    assetJO["Uuid"]             = _assetDTO.getUUID();
    assetJO["Name"]             = _assetDTO.getName();
    assetJO["CreationDatetime"] = _assetDTO.getCreationDateTime().toString(Qt::ISODateWithMs);
    assetJO["UpdateDatetime"]   = _assetDTO.getLastUpdateDateTime().toString(Qt::ISODateWithMs);
    assetJO["Path"]             = _assetDTO.getLoadedFile().getPath();

    QJsonDocument assetJDoc(assetJO);
    assetInfoFile.write(assetJDoc.toJson());
    assetInfoFile.close();
}

QString AssetBusiness::getAssetFileName() const
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (_assetDTO.getLoadedFile().getOriginalName().isEmpty()) {
        throw std::runtime_error("Loaded File is empty");
    }

    auto originalName = _assetDTO.getLoadedFile().getOriginalName();
    return originalName;
}

quint64 AssetBusiness::getAssetFileSize() const
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (_assetDTO.getDownloadedFileList().getDownloadedFileList().isEmpty()) {
        throw std::runtime_error("File list from asset downloaded is empty");
    }

    return _assetDTO.getDownloadedFileList().getDownloadedFileList().first().getSize();
}

} // namespace gcs
