#include "CurrentMissionResultBusiness.h"

#include <QsLog/QsLog.h>
#include <config.h>
#include <dataModel/RouteDTO.h>
#include <dataModel/dtos/CurrentMissionResultDTO.h>
#include <dataModel/dtos/InspectionTaskDTO.h>
#include <dataModel/dtos/MissionTaskDTO.h>

#include <QDir>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <random>

namespace gcs {

CurrentMissionResultBusiness::CurrentMissionResultBusiness(CurrentMissionResultDTO& currentMissionResultDTO) :
        _currentMissionResultDTO(currentMissionResultDTO)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

CurrentMissionResultBusiness::~CurrentMissionResultBusiness()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

void CurrentMissionResultBusiness::newMission(const QString& mainMissionsPath, const QString& inspectionPlanUUID)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QLOG_DEBUG() << __PRETTY_FUNCTION__ << "Current Mission Folder Path: " << mainMissionsPath;

    QDir mainMissionsDir(mainMissionsPath);
    if (!QDir().mkpath(mainMissionsDir.path())) {
        const auto msg = QString("Main missions directory has not been created: %1").arg(mainMissionsDir.path());
        throw std::runtime_error(msg.toStdString());
    }

    CurrentMissionResultDTO currentMissionResult;
    currentMissionResult.getCreationTimeStamp()   = QDateTime::currentDateTime();
    currentMissionResult.getLastUpdateTimeStamp() = QDateTime::currentDateTime();
    currentMissionResult.getInspectionPlanUUID()  = inspectionPlanUUID;
    currentMissionResult.getName()
            = QString("mission_%1").arg(currentMissionResult.getCreationTimeStamp().toString("yyyyMMddhhmmss"));

    QDir newMissionDir(mainMissionsDir.absoluteFilePath(currentMissionResult.getName()));
    if (!QDir().mkpath(newMissionDir.path())) {
        const auto msg = QString("New mission directory has not been created: %1").arg(newMissionDir.path());
        throw std::runtime_error(msg.toStdString());
    }

    QLOG_DEBUG() << __PRETTY_FUNCTION__ << "Successfully created new mission in folder:" << newMissionDir.path();

    currentMissionResult.getConsoleLogFilePath()          = newMissionDir.absoluteFilePath("event_log.csv");
    currentMissionResult.getTelemetryLogFilePath()        = newMissionDir.absoluteFilePath("path_followed.csv");
    currentMissionResult.getCheckListLogFilePath()        = newMissionDir.absoluteFilePath("checklist.csv");
    currentMissionResult.getLoadedTrajectoryLogFilePath() = newMissionDir.absoluteFilePath("planned_trajectory.csv");
    currentMissionResult.getMissionResultLogFilePath()    = newMissionDir.absoluteFilePath("mission_result.json");

    _currentMissionResultDTO = currentMissionResult;
    generateNewSyncId();

    saveMissionResultToJson();
}

void CurrentMissionResultBusiness::closeMission()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _currentMissionResultDTO = CurrentMissionResultDTO();
}

void CurrentMissionResultBusiness::saveLoadedRouteToFile(const RouteDTO& loadedRoute)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QString msg;
    msg += QString("## seq,task_uuid,pos_x,pos_y,pos_z,quat_x,quat_y,quat_z,quat_w\n");
    for (const auto& wpItem : loadedRoute.getWpsList()) {
        msg += QString("%1,%2,%3,%4,%5,%6,%7,%8,%9\n")
                       .arg(QString::number(wpItem.getId()))
                       .arg(wpItem.getTaskUUID())
                       .arg(QString::number(wpItem.getPosition().getX(), 'f', 3))
                       .arg(QString::number(wpItem.getPosition().getY(), 'f', 3))
                       .arg(QString::number(wpItem.getPosition().getZ(), 'f', 3))
                       .arg(QString::number(wpItem.getOrientation().getX(), 'f', 5))
                       .arg(QString::number(wpItem.getOrientation().getY(), 'f', 5))
                       .arg(QString::number(wpItem.getOrientation().getZ(), 'f', 5))
                       .arg(QString::number(wpItem.getOrientation().getW(), 'f', 5));
    }

    saveByteArrayDataInFile(_currentMissionResultDTO.getLoadedTrajectoryLogFilePath(), msg.toUtf8());
}

void CurrentMissionResultBusiness::saveCheckListToFile(const CheckListDTO& dto)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _currentMissionResultDTO.getCheckListDTO() = dto;

    QString msg;
    msg += QString("## index,name,despcription,status\n");
    for (const auto& item : _currentMissionResultDTO.getCheckListDTO().getCheckItemsList()) {
        msg += QString("%1,\"%2\",\"%3\",%4\n")
                       .arg(item.getIndex())
                       .arg(item.getName())
                       .arg(item.getDescription())
                       .arg(ToString(item.getCheckStatus()));
    }

    saveByteArrayDataInFile(_currentMissionResultDTO.getCheckListLogFilePath(), msg.toUtf8());
}

void CurrentMissionResultBusiness::saveMissionResultToJson()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    /// \note Timestamp format obtained from: https://datatracker.ietf.org/doc/html/rfc3339#section-5.6
    QJsonObject missionJObject;
    missionJObject["Name"]               = _currentMissionResultDTO.getName();
    missionJObject["SyncId"]             = _currentMissionResultDTO.getSyncId();
    missionJObject["InspectionPlanUuid"] = _currentMissionResultDTO.getInspectionPlanUUID();
    missionJObject["Description"]        = _currentMissionResultDTO.getDescription();
    missionJObject["CreationTimeStamp"]  = _currentMissionResultDTO.getCreationTimeStamp().toString(Qt::ISODateWithMs);
    missionJObject["LastUpdateTimeStamp"]
            = _currentMissionResultDTO.getLastUpdateTimeStamp().toString(Qt::ISODateWithMs);

    QJsonArray missionTasksJArray;
    for (const auto& missionTaskDTO : _currentMissionResultDTO.getMissionTaskList()) {
        QJsonObject missionTaskJO;
        missionTaskJO["Name"]                = missionTaskDTO.getName();
        missionTaskJO["Description"]         = missionTaskDTO.getDescription();
        missionTaskJO["InspTaskUuid"]        = missionTaskDTO.getInspectionTaskAssociated().getUUID();
        missionTaskJO["CreationTimeStamp"]   = missionTaskDTO.getCreationDateTime().toString(Qt::ISODateWithMs);
        missionTaskJO["LastUpdateTimeStamp"] = missionTaskDTO.getLastUpdateDateTime().toString(Qt::ISODateWithMs);

        missionTasksJArray.append(missionTaskJO);
    }
    missionJObject["MissionTasks"] = missionTasksJArray;

    QJsonDocument missionJDoc(missionJObject);

    saveByteArrayDataInFile(_currentMissionResultDTO.getMissionResultLogFilePath(), missionJDoc.toJson());
}

void CurrentMissionResultBusiness::updateMissionTask(const MissionTaskDTO& dto)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    auto it = std::find_if(
            _currentMissionResultDTO.getMissionTaskList().begin(),
            _currentMissionResultDTO.getMissionTaskList().end(),
            [&](MissionTaskDTO& missionTaskDTO) { return (missionTaskDTO.getId() == dto.getId()); });

    if (it == _currentMissionResultDTO.getMissionTaskList().end()) {
        const auto msg = QString("Mission Task %1 doesnt exist").arg(QString::number(it->getId()));
        QLOG_ERROR() << __PRETTY_FUNCTION__ << "-" << msg;
        throw std::runtime_error(msg.toStdString());
    }

    *it = dto;
}

void CurrentMissionResultBusiness::createMissionTaskListFromRoute(
        const RouteDTO& routeDTO, const QList<InspectionTaskDTO>& inspectionTaskList)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QList<MissionTaskDTO> missionTaskListDTO;
    QList<QString>        missionTaskUuidList;
    quint16               currentMissionIndex = 0;
    for (auto it = routeDTO.getWpsList().cbegin(); it != routeDTO.getWpsList().cend(); it++) {
        WaypointDTO waypoint = *it;
        const auto  iter     = std::find_if(
                inspectionTaskList.cbegin(), inspectionTaskList.cend(), [&](const InspectionTaskDTO inspDTO) {
                    return (inspDTO.getUUID() == waypoint.getTaskUUID());
                });

        if (iter == inspectionTaskList.cend()) {
            const QString msg("Waypoint contains taskUUID associated that there isnt in the Inspection Plan");
            QLOG_ERROR() << __PRETTY_FUNCTION__ << "-" << msg;
            throw std::runtime_error(msg.toStdString());
        }

        const auto missionTaskIndex = std::distance(routeDTO.getWpsList().cbegin(), it);

        if (missionTaskUuidList.contains(iter->getUUID())) {
            continue;
        }

        MissionTaskDTO missionTaskDTO;
        missionTaskDTO.getId()                       = missionTaskIndex;
        missionTaskDTO.getName()                     = QString("mission_task_%1").arg(currentMissionIndex++);
        missionTaskDTO.getCreationDateTime()         = QDateTime::currentDateTime();
        missionTaskDTO.getLastUpdateDateTime()       = QDateTime::currentDateTime();
        missionTaskDTO.getInspectionTaskAssociated() = *iter;

        missionTaskListDTO.push_back(missionTaskDTO);
        missionTaskUuidList.push_back(missionTaskDTO.getInspectionTaskAssociated().getUUID());
    }

    _currentMissionResultDTO.getMissionTaskList() = missionTaskListDTO;
}

void CurrentMissionResultBusiness::saveByteArrayDataInFile(const QString& fileName, const QByteArray& data)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (fileName.isEmpty() || fileName.isNull()) {
        throw std::runtime_error(QString("No valid log file path: %1").arg(fileName).toStdString());
    }

    QFile logFile(fileName);
    if (!logFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        throw std::runtime_error("Error opening log file");
    }

    QTextStream logTextStream(&logFile);
    logTextStream << data;
}

void CurrentMissionResultBusiness::generateNewSyncId()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    std::random_device                 r;
    std::default_random_engine         e1(r());
    std::uniform_int_distribution<int> uniform_dist(0, 2147483647);
    _currentMissionResultDTO.getSyncId() = uniform_dist(e1);
}

} // namespace gcs
