#include "MissionResultsBusiness.h"

#include <QsLog/QsLog.h>
#include <dataModel/dtos/MissionResultsDTO.h>

#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

namespace gcs {

MissionResultsBusiness::MissionResultsBusiness(MissionResultsDTO& missionResultsDTO) :
        _missionResultsDTO(missionResultsDTO)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

MissionResultsBusiness::~MissionResultsBusiness()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

void MissionResultsBusiness::loadMissionResults(const QString& mainMissionsPath)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (mainMissionsPath.isEmpty()) {
        throw std::runtime_error("Mission folders path is empty");
    }

    QDir mainMissionsDir(mainMissionsPath);
    if (!QDir().mkpath(mainMissionsDir.path())) {
        const auto msg = QString("Main missions directory has not been created: %1").arg(mainMissionsDir.path());
        throw std::runtime_error(msg.toStdString());
    }

    _missionResultsDTO.getFolderPath() = mainMissionsDir.path();

    const auto missionFolders
            = mainMissionsDir.entryList(QDir::NoDot | QDir::NoDotDot | QDir::Dirs, QDir::Time | QDir::Reversed);
    if (missionFolders.isEmpty()) {
        QLOG_INFO() << __PRETTY_FUNCTION__ << "There isnt mission inside folder:" << mainMissionsDir.path();
        return;
    }

    for (const auto& missionName : missionFolders) {
        QDir  missionDir(mainMissionsDir.absoluteFilePath(missionName));
        QFile missionResultFile(missionDir.absoluteFilePath("mission_result.json"));
        if (!missionResultFile.exists()) {
            throw std::runtime_error(
                    "Mission information file don't exists: " + missionResultFile.fileName().toStdString());
        }

        if (missionResultFile.size() <= 0) {
            QLOG_WARN() << __PRETTY_FUNCTION__ << "Mission result file is empty: " << missionResultFile.fileName();
            continue;
        }

        const auto currMissionDTO = convertMissionFileToDTO(missionResultFile.fileName());
        _missionResultsDTO.getList().push_back(currMissionDTO);
    }
}

void MissionResultsBusiness::removeMissionResult(const MissionResultDTO& misToRemove)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    createMissionBackUpFolder(misToRemove.getName());

    _missionResultsDTO.getList().removeOne(misToRemove);
}

void MissionResultsBusiness::obtainMissionResultList(
        const QStringList& missionNamesToUpload, QList<MissionResultDTO>& list) const
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    for (const auto& name : missionNamesToUpload) {
        const auto it = std::find_if(
                _missionResultsDTO.getList().cbegin(),
                _missionResultsDTO.getList().cend(),
                [&](const MissionResultDTO& missionResult) { return (missionResult.getName() == name); });

        if (it == _missionResultsDTO.getList().cend()) {
            const auto msg = QString("Mission Result (%1) does not exist in dataModel").arg(name);
            throw std::runtime_error(msg.toStdString());
        }

        list.push_back(*it);
    }
}

MissionResultDTO MissionResultsBusiness::convertMissionFileToDTO(const QString& missionInfoPath)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QFile missionInfoFile(missionInfoPath);
    if (!missionInfoFile.open(QIODevice::ReadOnly)) {
        const auto msg = QString("Could not open mission info file: %1").arg(missionInfoPath);
        throw std::runtime_error(msg.toStdString());
    }

    QByteArray    missionData = missionInfoFile.readAll();
    QJsonDocument missionJDoc(QJsonDocument::fromJson(missionData));

    const auto missionJObject = missionJDoc.object();

    _jsonCheckKeys.checkExistInspPlanBasicInfoKeys(missionJObject);
    MissionResultDTO missionResult;
    missionResult.getName()        = missionJObject["Name"].toString();
    missionResult.getDescription() = missionJObject["Description"].toString();
    missionResult.getSyncId()      = missionJObject["SyncId"].toInt();
    missionResult.getCreationTimeStamp()
            = QDateTime::fromString(missionJObject["CreationTimeStamp"].toString(), Qt::ISODateWithMs);
    missionResult.getLastUpdateTimeStamp()
            = QDateTime::fromString(missionJObject["LastUpdateTimeStamp"].toString(), Qt::ISODateWithMs);

    const auto missionTasksJArray = missionJObject["MissionTasks"].toArray();
    for (int i = 0; i < missionTasksJArray.size(); ++i) {
        auto missionTaskJO = missionTasksJArray[i].toObject();

        _jsonCheckKeys.checkExistMissionTaskKeys(missionTaskJO);
        MissionTaskDTO missionTask;
        missionTask.getId()          = i;
        missionTask.getName()        = missionTaskJO["Name"].toString();
        missionTask.getDescription() = missionTaskJO["Description"].toString();
        missionTask.getCreationDateTime()
                = QDateTime::fromString(missionTaskJO["CreationTimeStamp"].toString(), Qt::ISODateWithMs);
        missionTask.getLastUpdateDateTime()
                = QDateTime::fromString(missionTaskJO["LastUpdateTimeStamp"].toString(), Qt::ISODateWithMs);

        InspectionTaskDTO inspTaskDTO;
        inspTaskDTO.getUUID()                     = missionTaskJO["InspTaskUuid"].toString();
        missionTask.getInspectionTaskAssociated() = inspTaskDTO;

        missionResult.getMissionTaskList().append(missionTask);
    }

    return missionResult;
}

void MissionResultsBusiness::createMissionBackUpFolder(const QString& missionNameToSave)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QDir tempDir(_missionResultsDTO.getFolderPath());
    if (!tempDir.cdUp()) {
        const auto msg = QString("Do not exists cd up folder of: %1").arg(_missionResultsDTO.getFolderPath());
        throw std::runtime_error(msg.toStdString());
    }

    QDir missionsBackupDir = tempDir.absoluteFilePath("missions_backup");
    if (!QDir().mkpath(missionsBackupDir.path())) {
        const auto msg = QString("Missions backup directory has not been created: %1").arg(missionsBackupDir.path());
        throw std::runtime_error(msg.toStdString());
    }

    copyAndRemoveMissionFolder(
            QDir(_missionResultsDTO.getFolderPath()).absoluteFilePath(missionNameToSave),
            missionsBackupDir.absoluteFilePath(missionNameToSave));
}

void MissionResultsBusiness::copyAndRemoveMissionFolder(const QString& originPath, const QString& targetPath)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QDir targetDir(targetPath);
    if (!QDir().mkpath(targetDir.path())) {
        const auto msg = QString("Taget directory has not been created: %1").arg(targetDir.path());
        throw std::runtime_error(msg.toStdString());
    }

    QDir       originDir(originPath);
    const auto originFiles
            = originDir.entryList(QDir::NoDot | QDir::NoDotDot | QDir::Files, QDir::Time | QDir::Reversed);
    for (const auto& file : originFiles) {
        if (!QFile::copy(originDir.absoluteFilePath(file), targetDir.absoluteFilePath(file))) {
            const auto msg = QString("Error copying file: %1").arg(file);
            throw std::runtime_error(msg.toStdString());
        }
    }

    originDir.removeRecursively();
}

} // namespace gcs
