#include "MissionBusiness.h"

#include <QsLog/QsLog.h>
#include <config.h>

#include <QDateTime>
#include <QDir>

#include "../dtos/MissionDTO.h"

namespace gcs {

MissionBusiness::MissionBusiness(MissionDTO& missionDTO) :
        _missionDTO(missionDTO), _routeBusiness(missionDTO), _jsonWptsListFile(missionDTO)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

MissionBusiness::~MissionBusiness()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

void MissionBusiness::newMission(const QString& missionName)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QLOG_DEBUG() << __PRETTY_FUNCTION__
                 << " - "
                    "Name for the mission to create:"
                 << missionName;

    if (missionName.isEmpty()) {
        const QString msg("Mission name is empty");
        QLOG_ERROR() << __PRETTY_FUNCTION__ << "-" << msg;
        throw std::runtime_error(msg.toStdString());
    }

    BasicMissionDTO& basicMission = _missionDTO.getBasicMission();
    basicMission.getName()        = missionName;
}

void MissionBusiness::closeMission() Q_DECL_NOEXCEPT
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _missionDTO = MissionDTO();
}

void MissionBusiness::openMission(const QString& wptsListPath, const QString& inspPlanUUID)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QLOG_DEBUG() << __PRETTY_FUNCTION__ << " - Waypoint list path to open:" << wptsListPath;

    if (wptsListPath.isEmpty()) {
        const QString msg("Waypoint List path is empty");
        QLOG_ERROR() << __PRETTY_FUNCTION__ << " - " << msg;
        throw std::runtime_error(msg.toStdString());
    }

    _jsonWptsListFile.load(wptsListPath, inspPlanUUID);

    _routeBusiness.updateSeqNumbers();
    _routeBusiness.selectLastWaypoint();
}

void MissionBusiness::saveMission(const QString& inspPlanUUID, const QString& filePath)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    /// \note. Search the inspection plan folder. If not exist waypoint_lists folder, create it.
    QDir inspectionPlansDir(
            QDir::home().absoluteFilePath(QString(".pilotingGrcs/v%1.%2/inspectionPlans/inspectionPlan_%3/")
                                                  .arg(PROJECT_VERSION_MAJOR)
                                                  .arg(PROJECT_VERSION_MINOR)
                                                  .arg(inspPlanUUID)));
    if (!inspectionPlansDir.exists()) {
        const auto msg = QString("Directory of the Inspection plan with UUID: %1, not exists: %2")
                                 .arg(inspPlanUUID)
                                 .arg(inspectionPlansDir.path());
        throw std::runtime_error(msg.toStdString());
    }

    QDir wptsListsDir = inspectionPlansDir.absoluteFilePath("waypoint_lists");
    if (!QDir().mkpath(wptsListsDir.path())) {
        const auto msg = QString("Waypoint Lists directory has not been created: %1").arg(wptsListsDir.path());
        throw std::runtime_error(msg.toStdString());
    }

    auto currWptsListPath = QString();

    if (filePath.isEmpty()) {
        /// \note. Save current waypoint list with current timestamp as identifier
        currWptsListPath = wptsListsDir.absoluteFilePath(
                QString("waypoint_list_%1.json").arg(QDateTime::currentDateTime().toString("yyyyMMddhhmmss")));
    } else {
        currWptsListPath = filePath;

        if (QString::compare("json", QFileInfo(currWptsListPath).suffix()) != 0) {
            currWptsListPath.append(".json");
        }
    }

    _jsonWptsListFile.save(currWptsListPath, inspPlanUUID);
}

} // namespace gcs
