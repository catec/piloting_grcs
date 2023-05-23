
#include "MavsdkComms.h"

#include <QsLog/QsLog.h>
#include <common/CommonDefines.h>
#include <config.h>
#include <dataModel/ActionParametersDTO.h>
#include <dataModel/RobotAttitudeAngVelocityDTO.h>
#include <dataModel/RobotPositionVelocityNedDTO.h>
#include <dataModel/RouteDTO.h>
#include <dataModel/dtos/CommsLinkMavsdkDTO.h>
#include <dataModel/dtos/CommsProgressDTO.h>
#include <dataModel/dtos/CommsStatusDTO.h>
#include <dataModel/dtos/HomeDTO.h>
#include <dataModel/dtos/InspectionTaskDTO.h>
#include <dataModel/dtos/MissionDTO.h>
#include <dataModel/dtos/MsgConsolePopupDTO.h>
#include <mavsdk/mavlink_include.h>
#include <mavsdk/mavsdk.h>

#include <QApplication>
#include <QDir>
#include <QMap>
#include <QMessageBox>
#include <QQuaternion>
#include <QtConcurrent>
#include <chrono>
#include <future>
#include <iostream>
#include <thread>

#include "MavTypes.h"
#include "dtos/AlarmListDTO.h"
#include "dtos/AlarmStatusDTO.h"
#include "dtos/CheckListDTO.h"
#include "dtos/CommandLongDTO.h"
#include "dtos/CurrentInspItemDTO.h"
#include "dtos/HLActionListDTO.h"
#include "dtos/ReachedInspItemDTO.h"

namespace gcs {

MavsdkComms::MavsdkComms(QObject* parent) : QObject(parent), _heartbeat_timeout(false)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

MavsdkComms::~MavsdkComms()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

bool MavsdkComms::isConnected() const
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return _mavsdk && _mavsdk->is_connected();
}

void MavsdkComms::connectTo(QSharedPointer<CommsLinkDTO> idto)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        if (!idto) {
            throw std::runtime_error("DTO is NULL");
        }

        if (isConnected()) {
            throw std::runtime_error("Communication already established");
        }

        startProgressBar("Connecting to robotic system ...");
        auto dto = idto.dynamicCast<CommsLinkMavsdkDTO>();

        QtConcurrent::run(std::bind(&MavsdkComms::connectToRoboticSystem, this, std::placeholders::_1), *dto);
    } catch (std::exception& e) {
        QLOG_ERROR() << __PRETTY_FUNCTION__ << "-" << e.what();
        sendPopupMsg(
                tr("Robotic System"),
                MsgType::WARNING,
                tr("Cannot connect to robotic system due to: - ") + tr(e.what()));
    }
}

void MavsdkComms::connectToRoboticSystem(const CommsLinkMavsdkDTO& dto)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        if (!_mavsdk) {
            _mavsdk = std::make_unique<mavsdk::Mavsdk>();
        }

        const bool                    sendHeartbeatAlways = true;
        mavsdk::Mavsdk::Configuration mavsdk_config(
                dto.getLocalSystemId(),
                dto.getLocalComponentId(),
                sendHeartbeatAlways,
                mavsdk::Mavsdk::Configuration::UsageType::GroundStation);
        _mavsdk->set_configuration(mavsdk_config);

        std::this_thread::sleep_for(std::chrono::seconds(1));

        const auto connection_result = _mavsdk->setup_udp_connection(
                dto.getLocalIP().toStdString(),
                dto.getLocalPort(),
                dto.getTargetIP().toStdString(),
                dto.getTargetPort());
        if (connection_result != mavsdk::ConnectionResult::Success) {
            throw std::runtime_error(QString("Connection failed: %1").arg(ToString(connection_result)).toStdString());
        }

        sendConsoleMsg(MsgType::INFO, tr("Waiting 1 second to establish connection after define IPs and ports..."));

        std::this_thread::sleep_for(std::chrono::seconds(1));

        auto targetSystem = _mavsdk->system(dto.getTargetSystemId());
        if (!targetSystem) {
            _mavsdk.reset();
            throw std::runtime_error(QString("No target system (%1) alive").arg(dto.getTargetSystemId()).toStdString());
        }

        sendConsoleMsg(
                MsgType::INFO,
                QString("Target system (%1) connected!").arg(static_cast<unsigned>(targetSystem->get_system_id())));

        targetSystem->subscribe_is_connected([=](bool is_connected) {
            QLOG_TRACE() << __PRETTY_FUNCTION__;

            if (is_connected) {
                sendConsoleMsg(MsgType::INFO, tr("System has been discovered"));
            }

            auto dtoToCore              = createSharedDTO<CommsStatusDTO>();
            dtoToCore->getIsConnected() = is_connected;
            Q_EMIT sendDTOToCore(ActionType::UPDATE_COMMS_STATUS, QSharedPointer<IDTO>(dtoToCore));
        });

        if (_mavsdkTelemetry) {
            _mavsdkTelemetry.reset();
        }
        _mavsdkTelemetry = std::make_unique<mavsdk::Telemetry>(targetSystem);
        configureTelemetryCallbacks();

        if (_mavsdkAlarm) {
            _mavsdkAlarm.reset();
        }
        _mavsdkAlarm = std::make_unique<mavsdk::Alarm>(targetSystem);
        configureAlarmCallbacks();

        if (_mavsdkInspection) {
            _mavsdkInspection.reset();
        }
        _mavsdkInspection = std::make_unique<mavsdk::Inspection>(targetSystem);
        configureInspectionCallbacks();

        if (_mavsdkChecklist) {
            _mavsdkChecklist.reset();
        }
        _mavsdkChecklist = std::make_unique<mavsdk::Checklist>(targetSystem);

        if (_mavsdkHLAction) {
            _mavsdkHLAction.reset();
        }
        _mavsdkHLAction = std::make_unique<mavsdk::HLAction>(targetSystem);

        if (_mavsdkCommand) {
            _mavsdkCommand.reset();
        }
        _mavsdkCommand = std::make_unique<mavsdk::Command>(targetSystem);

        _mavsdk->register_on_timeout(std::bind(&MavsdkComms::heartbeatCb, this, std::placeholders::_1));

        sendConsoleMsg(MsgType::INFO, tr("Connection to robotics system successfully"));

        auto dtoToCore              = createSharedDTO<CommsStatusDTO>();
        dtoToCore->getIsConnected() = true;
        Q_EMIT sendDTOToCore(ActionType::UPDATE_COMMS_STATUS, QSharedPointer<IDTO>(dtoToCore));

        QMetaObject::invokeMethod(this, "finishProgressBar", Qt::QueuedConnection);
    } catch (std::exception& e) {
        QLOG_ERROR() << __PRETTY_FUNCTION__ << "-" << e.what();
        sendPopupMsg(
                tr("Robotic System"),
                MsgType::WARNING,
                tr("Cannot connect to robotic system due to: - ") + tr(e.what()));

        QMetaObject::invokeMethod(this, "finishProgressBar", Qt::QueuedConnection);
    }
}

void MavsdkComms::resetMavsdkModules()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    /// \note. This method is necessary because when modules are built and they all share ownership of the system
    ///         and, by extension, of the systemImpl.
    ///         Therefore you must destroy the other owners of system(systemImpl) before destroying mavsdk.
    if (_mavsdkTelemetry) {
        _mavsdkTelemetry.reset();
    }

    if (_mavsdkAlarm) {
        _mavsdkAlarm.reset();
    }

    if (_mavsdkInspection) {
        _mavsdkInspection.reset();
    }

    if (_mavsdkChecklist) {
        _mavsdkChecklist.reset();
    }

    if (_mavsdkHLAction) {
        _mavsdkHLAction.reset();
    }

    if (_mavsdkCommand) {
        _mavsdkCommand.reset();
    }

    if (_mavsdk) {
        _mavsdk.reset();
    }
}

void MavsdkComms::disconnectFrom()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        if (!isConnected()) {
            throw std::runtime_error("Communication not established");
        }

        resetMavsdkModules();

        sendConsoleMsg(MsgType::INFO, tr("Correctly disconnection from robotics system"));

        auto dtoToCore              = createSharedDTO<CommsStatusDTO>();
        dtoToCore->getIsConnected() = false;
        Q_EMIT sendDTOToCore(ActionType::UPDATE_COMMS_STATUS, QSharedPointer<IDTO>(dtoToCore));
    } catch (std::exception& e) {
        QLOG_ERROR() << __PRETTY_FUNCTION__ << "-" << e.what();
        sendPopupMsg(tr("Communications"), MsgType::WARNING, tr("Comms disconnect error: ") + e.what());
    }
}

void MavsdkComms::heartbeatCb(uint64_t system_id)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    sendConsoleMsg(MsgType::WARNING, tr("Heartbeat timeout from system: %1").arg(system_id));

    _heartbeat_timeout = true;

    auto dtoToCore              = createSharedDTO<CommsStatusDTO>();
    dtoToCore->getIsConnected() = false;
    Q_EMIT sendDTOToCore(ActionType::UPDATE_COMMS_STATUS, QSharedPointer<IDTO>(dtoToCore));
}

void MavsdkComms::checkIfHeartBeatTimedOut()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (_heartbeat_timeout) {
        _heartbeat_timeout = false;

        auto dtoToCore              = createSharedDTO<CommsStatusDTO>();
        dtoToCore->getIsConnected() = true;
        Q_EMIT sendDTOToCore(ActionType::UPDATE_COMMS_STATUS, QSharedPointer<IDTO>(dtoToCore));
    }
}

void MavsdkComms::configureTelemetryCallbacks()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (!_mavsdkTelemetry) {
        throw std::runtime_error("Telemetry object is null");
    }

    _mavsdkTelemetry->subscribe_position_velocity_ned([&](mavsdk::TelemetryBase::PositionVelocityNed data) {
        /// \note. Where is the best place to carry out this check?
        checkIfHeartBeatTimedOut();

        auto positionVelocityDTO             = createSharedDTO<RobotPositionVelocityNedDTO>();
        positionVelocityDTO->systemId()      = data.system_id;
        positionVelocityDTO->componentId()   = data.component_id;
        positionVelocityDTO->getPositionX()  = static_cast<double>(data.position.east_m);
        positionVelocityDTO->getPositionY()  = static_cast<double>(data.position.north_m);
        positionVelocityDTO->getPositionZ()  = -static_cast<double>(data.position.down_m);
        positionVelocityDTO->getNorthSpeed() = static_cast<float>(data.velocity.north_m_s);
        positionVelocityDTO->getEastSpeed()  = static_cast<float>(data.velocity.east_m_s);
        positionVelocityDTO->getDownSpeed()  = static_cast<float>(data.velocity.down_m_s);

        Q_EMIT sendDTOToCore(ActionType::UPDATE_POSITION_VELOCITY_NED, positionVelocityDTO);
    });

    _mavsdkTelemetry->subscribe_attitude([&](mavsdk::TelemetryBase::Attitude data) {
        auto attitudeDTO                       = createSharedDTO<RobotAttitudeAngVelocityDTO>();
        attitudeDTO->systemId()                = data.system_id;
        attitudeDTO->componentId()             = data.component_id;
        attitudeDTO->getQuaternionW()          = static_cast<double>(data.quaternion_angle.w);
        attitudeDTO->getQuaternionX()          = static_cast<double>(data.quaternion_angle.y);
        attitudeDTO->getQuaternionY()          = static_cast<double>(data.quaternion_angle.x);
        attitudeDTO->getQuaternionZ()          = -static_cast<double>(data.quaternion_angle.z);
        attitudeDTO->getRollAngularVelocity()  = static_cast<double>(data.angular_velocity.roll_rad_s);
        attitudeDTO->getPitchAngularVelocity() = static_cast<double>(data.angular_velocity.pitch_rad_s);
        attitudeDTO->getYawAngularVelocity()   = static_cast<double>(data.angular_velocity.yaw_rad_s);

        Q_EMIT sendDTOToCore(ActionType::UPDATE_ATTITUDE_ANGVELOCITY, attitudeDTO);
    });

    _mavsdkTelemetry->subscribe_text_status([&](mavsdk::TelemetryBase::TextStatus data) {
        /// \note. Where is the best place to carry out this check?
        checkIfHeartBeatTimedOut();

        std::stringstream out;
        out << data.text;
        switch (data.type) {
            case mavsdk::TelemetryBase::TextStatusType::Error: {
                sendConsoleMsg(MsgType::ERROR, QString::fromStdString(out.str()));
                break;
            }
            case mavsdk::TelemetryBase::TextStatusType::Warning: {
                sendConsoleMsg(MsgType::WARNING, QString::fromStdString(out.str()));
                break;
            }
            case mavsdk::TelemetryBase::TextStatusType::Info: {
                sendConsoleMsg(MsgType::INFO, QString::fromStdString(out.str()));
                break;
            }
            default: {
                sendConsoleMsg(MsgType::INFO, QString::fromStdString(out.str()));
                break;
            }
        }
    });
}

void MavsdkComms::configureAlarmCallbacks()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (!_mavsdkAlarm) {
        throw std::runtime_error("Alarm object is null");
    }

    _mavsdkAlarm->subscribe_alarm_status([&](mavsdk::AlarmBase::AlarmStatus data) {
        auto alarmStatusDTO = createSharedDTO<AlarmStatusDTO>();
        *alarmStatusDTO     = convertToDTO(data);

        sendDTOToCore(ActionType::UPDATE_ALARM_STATUS, alarmStatusDTO);
    });
}

void MavsdkComms::configureInspectionCallbacks()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (!_mavsdkInspection) {
        throw std::runtime_error("Inspection object is null");
    }

    _mavsdkInspection->subscribe_inspection_progress([&](mavsdk::InspectionBase::InspectionProgress progress) {
        auto currentInspItemDTO        = createSharedDTO<CurrentInspItemDTO>();
        currentInspItemDTO->getIndex() = progress.current;
        Q_EMIT sendDTOToCore(ActionType::UPDATE_CURRENT_INSP_ITEM, QSharedPointer<IDTO>(currentInspItemDTO));

        auto reachedInspItemDTO        = createSharedDTO<ReachedInspItemDTO>();
        reachedInspItemDTO->getIndex() = progress.reached;
        Q_EMIT sendDTOToCore(ActionType::UPDATE_REACHED_INSP_ITEM, QSharedPointer<IDTO>(reachedInspItemDTO));
    });
}

void MavsdkComms::sendWaypointsList(
        const QString&                  inspectionPlanUUID,
        const quint32&                  syncID,
        const MissionDTO&               mission,
        const QList<InspectionTaskDTO>& inspTasksList)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        if (!isConnected()) {
            throw std::runtime_error("Communication not established");
        }

        if (!_mavsdkInspection) {
            throw std::runtime_error("Inspection object is null");
        }

        /// \note It is assumed that the flight plan that is sent will be
        /// 		 the route that is selected
        validateRoute(mission);

        const RouteDTO& currentRoute = mission.getLocalRoutesMap()[mission.getCurrentRouteId()];

        // if(areEqualRoutes(mission.getLoadedRoute(), currentRoute))
        // 	throw std::runtime_error("Route is already loaded");

        const auto waypointList = convertRouteToWaypointList(inspectionPlanUUID, syncID, currentRoute, inspTasksList);

        startProgressBar("Uploading Waypoint List...");

        /// \note. This callback runs in separate thread
        auto cb = std::bind(&MavsdkComms::uploadInspectionCb, this, std::placeholders::_1, std::placeholders::_2);
        _mavsdkInspection->upload_inspection_async(waypointList, cb);
    } catch (const std::exception& e) {
        QLOG_ERROR() << __PRETTY_FUNCTION__ << "-" << e.what();
        sendPopupMsg(
                tr("Waypoint List"),
                MsgType::WARNING,
                tr("The waypoint list cannot be uploaded due to: ") + tr(e.what()));

        QMetaObject::invokeMethod(this, "finishProgressBar", Qt::QueuedConnection);
    }
}

void MavsdkComms::uploadInspectionCb(mavsdk::InspectionBase::Result result, mavsdk::InspectionBase::Ack ack)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        if (result != mavsdk::InspectionBase::Result::Success) {
            std::stringstream out;
            out << "Upload waypoint list error result: " << ToString(result);
            throw std::runtime_error(out.str());
        }

        if (ack != mavsdk::InspectionBase::Ack::Accepted) {
            std::stringstream out;
            out << "Upload waypoint list error ack: " << ToString(ack);
            throw std::runtime_error(out.str());
        }

        std::stringstream out;
        out << "Successfully waypoint list upload: " << ToString(ack);
        sendConsoleMsg(MsgType::INFO, QString::fromStdString(out.str()));

        /// \note. Send empty route to remove loaded one
        auto loadedRoute = createSharedDTO<RouteDTO>();
        Q_EMIT sendDTOToCore(ActionType::UPDATE_LOADED_ROUTE, QSharedPointer<IDTO>(loadedRoute));

        /// \note. Commented to ensure that waypoint list should be downloaded.
        // Q_EMIT sendDTOToCore(ActionType::UPDATE_LOADED_ROUTE_FROM_CURRENT_ROUTE);

        QMetaObject::invokeMethod(this, "finishProgressBar", Qt::QueuedConnection);
    } catch (const std::exception& e) {
        QLOG_ERROR() << __PRETTY_FUNCTION__ << "-" << e.what();
        sendPopupMsg(
                tr("Waypoint List"),
                MsgType::WARNING,
                tr("The waypoint list cannot be uploaded due to: ") + tr(e.what()));

        QMetaObject::invokeMethod(this, "finishProgressBar", Qt::QueuedConnection);
    }
}

void MavsdkComms::downloadWaypointsList()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        if (!isConnected()) {
            throw std::runtime_error("Communication not established");
        }

        if (!_mavsdkInspection) {
            throw std::runtime_error("Inspection object is null");
        }

        startProgressBar("Downloading Waypoints List...");

        auto cb = std::bind(&MavsdkComms::downloadInspectionCb, this, std::placeholders::_1, std::placeholders::_2);
        _mavsdkInspection->download_inspection_async(cb);

    } catch (const std::exception& e) {
        QLOG_ERROR() << __PRETTY_FUNCTION__ << "-" << e.what();
        sendPopupMsg(
                tr("Waypoint List"),
                MsgType::WARNING,
                tr("The waypoint list cannot be downloaded due to: ") + tr(e.what()));

        QMetaObject::invokeMethod(this, "finishProgressBar", Qt::QueuedConnection);
    }
}

void MavsdkComms::downloadInspectionCb(
        mavsdk::InspectionBase::Result result, mavsdk::InspectionBase::WaypointList waypointList)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        if (result != mavsdk::InspectionBase::Result::Success) {
            std::stringstream out;
            out << "Result obtaining waypoints error: " << result;
            throw std::runtime_error(out.str());
        }

        auto loadedRoute = createSharedDTO<RouteDTO>();
        *loadedRoute     = convertToDTO(waypointList);
        Q_EMIT sendDTOToCore(ActionType::UPDATE_LOADED_ROUTE, QSharedPointer<IDTO>(loadedRoute));

        sendConsoleMsg(
                MsgType::INFO,
                QString("Successfully waypoint list download with size: %1").arg(waypointList.items.size()));

        QMetaObject::invokeMethod(this, "finishProgressBar", Qt::QueuedConnection);
    } catch (const std::exception& e) {
        QLOG_ERROR() << __PRETTY_FUNCTION__ << "-" << e.what();
        sendPopupMsg(
                tr("Waypoint List"),
                MsgType::WARNING,
                tr("The waypoint list cannot be downloaded due to: ") + tr(e.what()));

        QMetaObject::invokeMethod(this, "finishProgressBar", Qt::QueuedConnection);
    }
}

void MavsdkComms::setCurrentWaypointItem(const quint16 wp_idx)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        if (!isConnected()) {
            throw std::runtime_error("Communication not established");
        }

        if (!_mavsdkInspection) {
            throw std::runtime_error("Inspection object is null");
        }

        QLOG_DEBUG() << __PRETTY_FUNCTION__ << "Setting current inspecton item to: " << wp_idx;
        _mavsdkInspection->set_current_inspection_item(wp_idx);

        sendConsoleMsg(
                MsgType::INFO, QString::fromStdString("Successfully changed current waypoint item to: %1").arg(wp_idx));
    } catch (const std::exception& e) {
        QLOG_ERROR() << __PRETTY_FUNCTION__ << "-" << e.what();
        sendPopupMsg(
                tr("Waypoint Item"), MsgType::WARNING, tr("Set current waypoint item error due to: ") + tr(e.what()));
    }
}

void MavsdkComms::downloadCheckList()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        if (!isConnected()) {
            throw std::runtime_error("Communication not established");
        }

        if (!_mavsdkChecklist) {
            throw std::runtime_error("CheckList object is null");
        }

        startProgressBar("Downloading Check List...");

        auto cb = std::bind(&MavsdkComms::downloadCheckListCb, this, std::placeholders::_1, std::placeholders::_2);
        _mavsdkChecklist->download_checklist_async(cb);
    } catch (const std::exception& e) {
        QLOG_ERROR() << __PRETTY_FUNCTION__ << "-" << e.what();
        sendPopupMsg(
                tr("CheckList"), MsgType::WARNING, tr("The CheckList cannot be downloaded due to: ") + tr(e.what()));

        QMetaObject::invokeMethod(this, "finishProgressBar", Qt::QueuedConnection);
    }
}

void MavsdkComms::downloadCheckListCb(mavsdk::ChecklistBase::Result result, mavsdk::ChecklistBase::Checklist checklist)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        if (result != mavsdk::ChecklistBase::Result::Success) {
            std::stringstream out;
            out << "Obtaining checklist error: " << ToString(result);
            throw std::runtime_error(out.str());
        }

        if (checklist.items.empty()) {
            throw std::runtime_error("Empty downloaded checklist");
        }

        auto checkListDTO = createSharedDTO<CheckListDTO>();
        *checkListDTO     = convertToDTO(checklist);
        sendDTOToCore(ActionType::RECEIVE_CHECKLIST, checkListDTO);

        sendPopupMsg(
                tr("CheckList"),
                MsgType::INFO,
                tr("The CheckList has been successfully downloaded with a valid format."));

        QMetaObject::invokeMethod(this, "finishProgressBar", Qt::QueuedConnection);
    } catch (const std::exception& e) {
        QLOG_ERROR() << __PRETTY_FUNCTION__ << "-" << e.what();
        sendPopupMsg(
                tr("CheckList"), MsgType::WARNING, tr("The CheckList cannot be downloaded due to: ") + tr(e.what()));
        QMetaObject::invokeMethod(this, "finishProgressBar", Qt::QueuedConnection);
    }
}

void MavsdkComms::downloadAlarmsList()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        if (!isConnected()) {
            throw std::runtime_error("Communication not established");
        }

        if (!_mavsdkAlarm) {
            throw std::runtime_error("Alarms object is null");
        }

        startProgressBar("Downloading Alarm List...");

        auto cb = std::bind(&MavsdkComms::downloadAlarmListCb, this, std::placeholders::_1, std::placeholders::_2);
        _mavsdkAlarm->download_alarm_async(cb);
    } catch (const std::exception& e) {
        QLOG_ERROR() << __PRETTY_FUNCTION__ << "-" << e.what();
        sendPopupMsg(
                tr("Alarm List"), MsgType::WARNING, tr("The alarm list cannot be downloaded due to: ") + tr(e.what()));

        QMetaObject::invokeMethod(this, "finishProgressBar", Qt::QueuedConnection);
    }
}

void MavsdkComms::downloadAlarmListCb(mavsdk::AlarmBase::Result result, mavsdk::AlarmBase::AlarmList alarmList)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        if (result != mavsdk::AlarmBase::Result::Success) {
            std::stringstream out;
            out << "Obtaining alarms error: " << ToString(result);
            throw std::runtime_error(out.str());
        }

        if (alarmList.items.empty()) {
            throw std::runtime_error("Empty downloaded alarms list");
        }

        auto alarmListDTO = createSharedDTO<AlarmListDTO>();
        *alarmListDTO     = convertToDTO(alarmList);
        sendDTOToCore(ActionType::RECEIVE_ALARMS_LIST, alarmListDTO);

        sendPopupMsg(
                tr("Alarm List"),
                MsgType::INFO,
                tr("The alarm list has been successfully downloaded with a valid format."));

        QMetaObject::invokeMethod(this, "finishProgressBar", Qt::QueuedConnection);
    } catch (const std::exception& e) {
        QLOG_ERROR() << __PRETTY_FUNCTION__ << "-" << e.what();
        sendPopupMsg(
                tr("Alarm List"), MsgType::WARNING, tr("The alarm list cannot be downloaded due to: ") + tr(e.what()));
        QMetaObject::invokeMethod(this, "finishProgressBar", Qt::QueuedConnection);
    }
}

void MavsdkComms::downloadHighLevelActions()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        if (!isConnected()) {
            throw std::runtime_error("Communication not established");
        }

        if (!_mavsdkHLAction) {
            throw std::runtime_error("HL Actions object is null");
        }

        startProgressBar("Downloading high level actions...");
        auto cb = std::bind(
                &MavsdkComms::downloadHighLevelActionsCb, this, std::placeholders::_1, std::placeholders::_2);
        _mavsdkHLAction->download_hl_action_async(cb);
    } catch (const std::exception& e) {
        QLOG_ERROR() << __PRETTY_FUNCTION__ << "-" << e.what();
        sendPopupMsg(
                tr("High Level Actions"),
                MsgType::WARNING,
                tr("The high level actions cannot be downloaded due to: ") + tr(e.what()));

        QMetaObject::invokeMethod(this, "finishProgressBar", Qt::QueuedConnection);
    }
}

void MavsdkComms::downloadHighLevelActionsCb(
        mavsdk::HLActionBase::Result result, mavsdk::HLActionBase::HLActionList hlActionList)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        if (result != mavsdk::HLActionBase::Result::Success) {
            std::stringstream out;
            out << "Result obtaining HL Actions: " << ToString(result);
            throw std::runtime_error(out.str());
        }

        if (hlActionList.items.empty()) {
            throw std::runtime_error("Empty downloaded high level actions");
        }

        auto hlActionDTO = createSharedDTO<HLActionListDTO>();
        *hlActionDTO     = convertToDTO(hlActionList);
        sendDTOToCore(ActionType::RECEIVE_HLACTION_LIST, hlActionDTO);

        sendPopupMsg(
                tr("High Level Action"),
                MsgType::INFO,
                tr("The high level actions have been downloaded successfully with valid format."));

        QMetaObject::invokeMethod(this, "finishProgressBar", Qt::QueuedConnection);
    } catch (const std::exception& e) {
        QLOG_ERROR() << __PRETTY_FUNCTION__ << "-" << e.what();
        sendPopupMsg(
                tr("High Level Actions"),
                MsgType::WARNING,
                tr("The high level actions cannot be downloaded due to: ") + tr(e.what()));
        QMetaObject::invokeMethod(this, "finishProgressBar", Qt::QueuedConnection);
    }
}

void MavsdkComms::sendHome(QSharedPointer<HomeDTO> homeDTO)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        if (!homeDTO) {
            throw std::runtime_error("DTO is null");
        }

        auto dto            = createSharedDTO<CommandLongDTO>();
        dto->getId()        = MAV_CMD_DO_SET_HOME_QUATERNION;
        dto->getParamList() = convertToParamListDTO(*homeDTO);

        sendCommand(dto);
    } catch (const std::exception& e) {
        QLOG_ERROR() << __PRETTY_FUNCTION__ << "-" << e.what();
        sendPopupMsg(tr("Send Home"), MsgType::WARNING, tr("Send home to vehicle error: ") + tr(e.what()));
    }
}

void MavsdkComms::sendCommand(QSharedPointer<CommandLongDTO> dto)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        if (!isConnected()) {
            throw std::runtime_error("Communication not established");
        }

        if (!_mavsdkCommand) {
            throw std::runtime_error("Commands object is null");
        }

        if (!dto) {
            throw std::runtime_error("DTO is null");
        }

        startProgressBar("Sending command to vehicle...");
        auto cb = std::bind(&MavsdkComms::sendCommandCb, this, std::placeholders::_1, std::placeholders::_2);
        _mavsdkCommand->send_command_async(convertFromDTO(*dto), cb);
    } catch (const std::exception& e) {
        QLOG_ERROR() << __PRETTY_FUNCTION__ << "-" << e.what();
        sendPopupMsg(tr("Commmand"), MsgType::WARNING, tr("Send command to vehicle error: ") + tr(e.what()));

        QMetaObject::invokeMethod(this, "finishProgressBar", Qt::QueuedConnection);
    }
}

void MavsdkComms::sendCommandCb(mavsdk::CommandBase::Result result, mavsdk::CommandBase::CommandAck ack)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        if (result == mavsdk::CommandBase::Result::InProgress) {
            sendConsoleMsg(MsgType::INFO, "Command processing in progress ...");
            return;
        }

        if (result != mavsdk::CommandBase::Result::Success) {
            std::stringstream out;
            out << "Sending command to vehicle result: " << ToString(result);
            throw std::runtime_error(out.str());
        }

        /// \note. ack.progress: Also used as result_param1, it can be set with an enum containing the errors reasons of
        /// why the command
        ///         was denied, or the progress percentage when result is MAV_RESULT_IN_PROGRESS (255 if the progress is
        ///         unknown).
        /// \note. ack.result_param2: Additional parameter of the result, example: which parameter of
        /// MAV_CMD_NAV_WAYPOINT caused it to be denied.

        if (ack.result != MAV_RESULT_ACCEPTED && ack.result != MAV_RESULT_IN_PROGRESS) {
            throw std::runtime_error(QString("Sended command: %1 to vehicle. Mav Result received: %2. With ACK "
                                             "progress: %3 and result_param2: %4")
                                             .arg(ack.command)
                                             .arg(ToString(static_cast<MAV_RESULT>(ack.result)))
                                             .arg(ack.progress)
                                             .arg(ack.result_param2)
                                             .toStdString());
        }

        sendConsoleMsg(MsgType::INFO, "Sucessfully sent command to vehicle");
        QMetaObject::invokeMethod(this, "finishProgressBar", Qt::QueuedConnection);
    } catch (const std::exception& e) {
        QLOG_ERROR() << __PRETTY_FUNCTION__ << "-" << e.what();
        sendPopupMsg(tr("Commmand"), MsgType::WARNING, tr("Send command to vehicle error: ") + tr(e.what()));
        QMetaObject::invokeMethod(this, "finishProgressBar", Qt::QueuedConnection);
    }
}

void MavsdkComms::validateRoute(const MissionDTO& mission)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (mission.getLocalRoutesMap().isEmpty()) {
        const QString msg("No route in the mission");
        QLOG_ERROR() << "MavsdkComms::validateRoute() -" << msg;
        throw std::runtime_error(msg.toStdString());
    }

    const QMap<int, RouteDTO>& routesMap = mission.getLocalRoutesMap();
    if (!routesMap.contains(mission.getCurrentRouteId())) {
        const QString msg("There is not a current route");
        QLOG_ERROR() << "MavsdkComms::validateRoute() -" << msg;
        throw std::runtime_error(msg.toStdString());
    }

    /// \note. Commented because not all robots require home
    // const HomeDTO &loadedHome = mission.getLoadedHome();
    // if(!loadedHome.getIsEnabled())
    // {
    //    const QString msg("Loaded home has not been set yet");
    //    QLOG_ERROR() << "MavsdkComms::validateRoute() -" << msg;
    //    throw std::runtime_error(msg.toStdString());
    // }
}

bool MavsdkComms::areEqualRoutes(const RouteDTO& loadedRoute, const RouteDTO& route)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    const auto& loadedWpsList = loadedRoute.getWpsList();
    const auto& wpsList       = route.getWpsList();

    if (loadedWpsList.size() != wpsList.size()) {
        return false;
    }

    for (int i = 0; i < loadedWpsList.size(); ++i) {
        if (loadedWpsList.at(i).getPosition() != wpsList.at(i).getPosition()
            || loadedWpsList.at(i).getOrientation() != wpsList.at(i).getOrientation()) {
            return false;
        }
    }

    return true;
}

mavsdk::InspectionBase::WaypointList MavsdkComms::convertRouteToWaypointList(
        const QString&                  inspectionPlanUUID,
        const quint32&                  syncID,
        const RouteDTO&                 route,
        const QList<InspectionTaskDTO>& inspTasksList)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    mavsdk::InspectionBase::WaypointList waypoints_list{};
    waypoints_list.plan_uuid = inspectionPlanUUID.toStdString();
    waypoints_list.sync_id   = syncID;
    for (const auto& wp : route.getWpsList()) {
        if (wp.getWpType() == WaypointType::POSE) {
            mavsdk::InspectionBase::WaypointItem wp_item{};
            wp_item.command      = MAV_CMD::MAV_CMD_NAV_WAYPOINT_QUATERNION;
            wp_item.autocontinue = static_cast<uint8_t>(wp.getAutoContinue() ? 1 : 0);
            wp_item.task_uuid    = wp.getTaskUUID().toStdString();
            wp_item.param1       = wp.getOrientation().getW();
            wp_item.param2       = wp.getOrientation().getX();
            wp_item.param3       = wp.getOrientation().getY();
            wp_item.param4       = wp.getOrientation().getZ();
            wp_item.x            = wp.getPosition().getX();
            wp_item.y            = wp.getPosition().getY();
            wp_item.z            = wp.getPosition().getZ();
            waypoints_list.items.push_back(wp_item);
        } else if (wp.getWpType() == WaypointType::ACTION) {
            auto it = std::find_if(
                    inspTasksList.cbegin(), inspTasksList.cend(), [&](const InspectionTaskDTO& inspTask) {
                        return (wp.getTaskUUID() == inspTask.getUUID());
                    });
            if (it == inspTasksList.cend()) {
                const auto msg = QString("The waypoint task associated not match with any inspection task UUID: %1")
                                         .arg(wp.getTaskUUID());
                throw std::runtime_error(msg.toStdString());
            }

            const auto currInspTask = *it;
            if (currInspTask.getLocationType() == InspectionTaskLocationType::POINT) {
                convertInspectionPointToWaypoints(
                        currInspTask, wp.getAutoContinue(), wp.getActionParameters(), waypoints_list);
            } else if (currInspTask.getLocationType() == InspectionTaskLocationType::AREA) {
                convertInspectionAreaToWaypoints(
                        currInspTask, wp.getAutoContinue(), wp.getActionParameters(), waypoints_list);
            } else {
                const auto msg = QString("Invalid inspection task type: %1")
                                         .arg(static_cast<uint8_t>(currInspTask.getLocationType()));
                throw std::runtime_error(msg.toStdString());
            }
        } else {
            throw std::runtime_error("Unknown waypoint type");
        }
    }

    return waypoints_list;
}

void MavsdkComms::convertInspectionPointToWaypoints(
        const InspectionTaskDTO&              currInspTask,
        const bool&                           autocontinue,
        const ActionParametersDTO&            actionParameters,
        mavsdk::InspectionBase::WaypointList& wptsList)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    mavsdk::InspectionBase::WaypointItem pt_center_wp_item{};
    pt_center_wp_item.command        = MAV_CMD::MAV_CMD_NAV_INSP_POINT;
    pt_center_wp_item.autocontinue   = static_cast<uint8_t>(autocontinue ? 1 : 0);
    pt_center_wp_item.task_uuid      = currInspTask.getUUID().toStdString();
    pt_center_wp_item.task_type_uuid = currInspTask.getType().getUUID().toStdString();
    pt_center_wp_item.param1         = actionParameters.getParam1();
    pt_center_wp_item.param2         = actionParameters.getParam2();
    pt_center_wp_item.param3         = actionParameters.getParam3();
    pt_center_wp_item.param4         = actionParameters.getParam4();
    pt_center_wp_item.x              = currInspTask.getPosition().getX();
    pt_center_wp_item.y              = currInspTask.getPosition().getY();
    pt_center_wp_item.z              = currInspTask.getPosition().getZ();
    wptsList.items.push_back(pt_center_wp_item);
}

QGenericMatrix<3, 3, float> getRotationZYXMatrix(const float& roll, const float& pitch, const float& yaw)
{
    float RxValues[] = {1.0, 0.0, 0.0, 0.0, std::cos(roll), -std::sin(roll), 0.0, std::sin(roll), std::cos(roll)};
    QGenericMatrix<3, 3, float> Rx(RxValues);

    float RyValues[] = {std::cos(pitch), 0.0, std::sin(pitch), 0.0, 1.0, 0.0, -std::sin(pitch), 0.0, std::cos(pitch)};
    QGenericMatrix<3, 3, float> Ry(RyValues);

    float RzValues[] = {std::cos(yaw), -std::sin(yaw), 0.0, std::sin(yaw), std::cos(yaw), 0.0, 0.0, 0.0, 1.0};
    QGenericMatrix<3, 3, float> Rz(RzValues);

    return Rz * Ry * Rx;
}

void rotatePoints(
        const float&        roll,
        const float&        pitch,
        const float&        yaw,
        const QVector3D     originPt,
        QVector<QVector3D>& ptsToRotate)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    const auto rotationMatrix = getRotationZYXMatrix(roll, pitch, yaw);

    for (auto& pt : ptsToRotate) {
        float                       ptValues[] = {pt.x() - originPt.x(), pt.y() - originPt.y(), pt.z() - originPt.z()};
        QGenericMatrix<1, 3, float> localPoint(ptValues);

        QGenericMatrix<1, 3, float> rotatedPt = rotationMatrix * localPoint;

        pt.setX(originPt.x() + rotatedPt(0, 0));
        pt.setY(originPt.y() + rotatedPt(1, 0));
        pt.setZ(originPt.z() + rotatedPt(2, 0));
    }
}

/// \note. The order of waypoints is the following:
///   1 _________ 4
///    |         |
///    |         |
///    |         |
///    |_________|
///   2           3

void MavsdkComms::convertInspectionAreaToWaypoints(
        const InspectionTaskDTO&              currInspTask,
        const bool&                           autocontinue,
        const ActionParametersDTO&            actionParameters,
        mavsdk::InspectionBase::WaypointList& wptsList)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    /// \note. Define plane corners in the order defined previously
    QVector<QVector3D> areaCorners(
            {QVector3D(
                     currInspTask.getPosition().getX() - (currInspTask.getWidth() / 2.0f),
                     currInspTask.getPosition().getY(),
                     currInspTask.getPosition().getZ() + (currInspTask.getHeight() / 2.0f)),
             QVector3D(
                     currInspTask.getPosition().getX() - (currInspTask.getWidth() / 2.0f),
                     currInspTask.getPosition().getY(),
                     currInspTask.getPosition().getZ() - (currInspTask.getHeight() / 2.0f)),
             QVector3D(
                     currInspTask.getPosition().getX() + (currInspTask.getWidth() / 2.0f),
                     currInspTask.getPosition().getY(),
                     currInspTask.getPosition().getZ() - (currInspTask.getHeight() / 2.0f)),
             QVector3D(
                     currInspTask.getPosition().getX() + (currInspTask.getWidth() / 2.0f),
                     currInspTask.getPosition().getY(),
                     currInspTask.getPosition().getZ() + (currInspTask.getHeight() / 2.0f))});

    const auto areaOrigin = QVector3D(
            currInspTask.getPosition().getX(), currInspTask.getPosition().getY(), currInspTask.getPosition().getZ());

    const auto ea = ToEulerAngles(
            currInspTask.getOrientation().getW(),
            currInspTask.getOrientation().getX(),
            currInspTask.getOrientation().getY(),
            currInspTask.getOrientation().getZ());
    rotatePoints(ea[0], ea[1], ea[2], areaOrigin, areaCorners);

    for (const auto& cornerPt : areaCorners) {
        mavsdk::InspectionBase::WaypointItem cornerWpItem{};
        cornerWpItem.command        = MAV_CMD::MAV_CMD_NAV_INSP_AREA;
        cornerWpItem.autocontinue   = static_cast<uint8_t>(autocontinue ? 1 : 0);
        cornerWpItem.task_uuid      = currInspTask.getUUID().toStdString();
        cornerWpItem.task_type_uuid = currInspTask.getType().getUUID().toStdString();
        cornerWpItem.param1         = actionParameters.getParam1();
        cornerWpItem.param2         = actionParameters.getParam2();
        cornerWpItem.param3         = actionParameters.getParam3();
        cornerWpItem.param4         = actionParameters.getParam4();
        cornerWpItem.x              = cornerPt.x();
        cornerWpItem.y              = cornerPt.y();
        cornerWpItem.z              = cornerPt.z();
        wptsList.items.push_back(cornerWpItem);
    }
}

void MavsdkComms::sendConsoleMsg(const MsgType& type, const QString& msg)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    auto msgConsoleDTO          = createSharedDTO<MsgConsoleDTO>();
    msgConsoleDTO->getMsgType() = type;
    msgConsoleDTO->getMessage() = msg;
    Q_EMIT sendDTOToCore(ActionType::UPDATE_MSG_CONSOLE, QSharedPointer<IDTO>(msgConsoleDTO));
}

void MavsdkComms::sendPopupMsg(const QString& title, const MsgType& type, const QString& msg)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    auto msgConsoleDTO          = createSharedDTO<MsgConsolePopupDTO>();
    msgConsoleDTO->getTitle()   = title;
    msgConsoleDTO->getMsgType() = type;
    msgConsoleDTO->getMessage() = msg;
    Q_EMIT sendDTOToCore(ActionType::UPDATE_MSG_CONSOLE, QSharedPointer<IDTO>(msgConsoleDTO));
}

void MavsdkComms::startProgressBar(const QString& msg)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    auto dtoToCore                    = createSharedDTO<CommsProgressDTO>();
    dtoToCore->getCommsProgressType() = CommsProgressType::START;
    dtoToCore->getCommsProgressInfo() = msg;
    sendDTOToCore(ActionType::UPDATE_COMMS_PROGRESS, QSharedPointer<IDTO>(dtoToCore));
}

void MavsdkComms::finishProgressBar()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    /// \note Wait a bit, for fast transactions when there is no time to launch the START.
    auto dieTime = QTime::currentTime().addMSecs(100);
    while (QTime::currentTime() < dieTime) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
    }

    auto dtoToCore                    = createSharedDTO<CommsProgressDTO>();
    dtoToCore->getCommsProgressType() = CommsProgressType::STOP;
    sendDTOToCore(ActionType::UPDATE_COMMS_PROGRESS, QSharedPointer<IDTO>(dtoToCore));
}

mavsdk::CommandBase::CommandLong MavsdkComms::convertFromDTO(const CommandLongDTO& commandLongDTO)
{
    mavsdk::CommandBase::CommandLong command;
    command.command      = commandLongDTO.getId();
    command.confirmation = commandLongDTO.getConfirmation();

    command.params.param1 = getParamValueFromIndex(1, commandLongDTO.getParamList());
    command.params.param2 = getParamValueFromIndex(2, commandLongDTO.getParamList());
    command.params.param3 = getParamValueFromIndex(3, commandLongDTO.getParamList());
    command.params.param4 = getParamValueFromIndex(4, commandLongDTO.getParamList());
    command.params.param5 = getParamValueFromIndex(5, commandLongDTO.getParamList());
    command.params.param6 = getParamValueFromIndex(6, commandLongDTO.getParamList());
    command.params.param7 = getParamValueFromIndex(7, commandLongDTO.getParamList());

    return command;
}

float MavsdkComms::getParamValueFromIndex(const qint16& index, const QList<ParamDTO>& paramList)
{
    float value;
    auto  it = std::find_if(paramList.cbegin(), paramList.cend(), [&](const ParamDTO& paramDTO) {
        return (paramDTO.getIndex() == index);
    });

    if (it != paramList.cend()) {
        value = it->getValue();
    } else {
        sendConsoleMsg(
                MsgType::WARNING,
                QString("The command to be sent contains does not contain parameter %1. Make sure that you want to "
                        "send this command.")
                        .arg(static_cast<int>(index)));
        throw std::runtime_error(
                QString("Param: %1 does not exists in ParamList").arg(static_cast<int>(index)).toStdString());
    }

    return value;
}

CheckListDTO MavsdkComms::convertToDTO(const mavsdk::ChecklistBase::Checklist& checklist)
{
    QList<CheckListItemDTO> itemList;
    for (const auto& checkitem : checklist.items) {
        if (checkitem.name.length() > MAVLINK_MSG_CHECK_LIST_ITEM_FIELD_NAME_LEN) {
            throw std::runtime_error("Checklist item name too long");
        } else if (checkitem.description.length() > MAVLINK_MSG_CHECK_LIST_ITEM_FIELD_DESCRIPTION_LEN) {
            throw std::runtime_error("Checklist item description too long");
        }

        CheckListItemDTO checklistItem;
        checklistItem.getIndex()       = checkitem.index;
        checklistItem.getName()        = QString::fromStdString(checkitem.name);
        checklistItem.getDescription() = QString::fromStdString(checkitem.description);
        itemList.push_back(checklistItem);
    }

    CheckListDTO checkListDTO;
    checkListDTO.getCheckItemsList() = itemList;

    return checkListDTO;
}

HLActionListDTO MavsdkComms::convertToDTO(const mavsdk::HLActionBase::HLActionList& hlActionList)
{
    QList<HLActionItemDTO> itemList;
    for (const auto& hlItem : hlActionList.items) {
        if (hlItem.name.length() > MAVLINK_MSG_HL_ACTION_LIST_ITEM_FIELD_NAME_LEN) {
            throw std::runtime_error("HLAction item name too long");
        } else if (hlItem.description.length() > MAVLINK_MSG_HL_ACTION_LIST_ITEM_FIELD_DESCRIPTION_LEN) {
            throw std::runtime_error("HLAction item description too long");
        }

        HLActionItemDTO hlActionItem;
        hlActionItem.getIndex()                     = hlItem.index;
        hlActionItem.getName()                      = hlItem.name.c_str();
        hlActionItem.getDescription()               = hlItem.description.c_str();
        hlActionItem.getAssociatedCommand().getId() = hlItem.command;
        itemList.push_back(hlActionItem);
    }

    HLActionListDTO hlActionListDTO;
    hlActionListDTO.getHLActionList() = itemList;

    return hlActionListDTO;
}

AlarmListDTO MavsdkComms::convertToDTO(const mavsdk::AlarmBase::AlarmList& alarmList)
{
    QList<AlarmItemDTO> itemList;
    for (const auto& alarmItem : alarmList.items) {
        if (alarmItem.name.length() > MAVLINK_MSG_ALARM_LIST_ITEM_FIELD_NAME_LEN) {
            throw std::runtime_error("HLAction item name too long");
        } else if (alarmItem.description.length() > MAVLINK_MSG_ALARM_LIST_ITEM_FIELD_DESCRIPTION_LEN) {
            throw std::runtime_error("HLAction item description too long");
        }

        AlarmItemDTO alarmItemDTO;
        alarmItemDTO.getIndex()       = alarmItem.index;
        alarmItemDTO.getName()        = alarmItem.name.c_str();
        alarmItemDTO.getDescription() = alarmItem.description.c_str();
        itemList.push_back(alarmItemDTO);
    }

    AlarmListDTO alarmListDTO;
    alarmListDTO.getAlarmList() = itemList;

    return alarmListDTO;
}

AlarmStatusDTO MavsdkComms::convertToDTO(const mavsdk::AlarmBase::AlarmStatus& alarmStatus)
{
    AlarmStatusDTO alarmStatusDTO;
    alarmStatusDTO.getStampMs()       = alarmStatus.stamp_ms;
    alarmStatusDTO.getIndex()         = alarmStatus.index;
    alarmStatusDTO.getAlarmStatus()   = static_cast<AlarmStatusType>(alarmStatus.status);
    alarmStatusDTO.getErrorsCount()   = alarmStatus.errors_count;
    alarmStatusDTO.getWarningsCount() = alarmStatus.warns_count;

    return alarmStatusDTO;
}

void appendParameterToList(const int& index, const float& value, QList<ParamDTO>& paramList)
{
    ParamDTO paramDTO;
    paramDTO.getIndex() = index;
    paramDTO.getValue() = value;
    paramList.push_back(paramDTO);
}

QList<ParamDTO> MavsdkComms::convertToParamListDTO(const HomeDTO& homeDTO)
{
    std::vector<float> quat = degToQuaternion(0, 0, homeDTO.getYawOrientation());

    QList<ParamDTO> paramList;
    appendParameterToList(1, quat[0], paramList);
    appendParameterToList(2, quat[1], paramList);
    appendParameterToList(3, quat[2], paramList);
    appendParameterToList(4, quat[3], paramList);
    appendParameterToList(5, homeDTO.getPosition().getX(), paramList);
    appendParameterToList(6, homeDTO.getPosition().getY(), paramList);
    appendParameterToList(7, homeDTO.getPosition().getZ(), paramList);

    return paramList;
}

RouteDTO MavsdkComms::convertToDTO(const mavsdk::InspectionBase::WaypointList& receivedWptsList)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    RouteDTO receivedRoute;
    /// \todo. Take from msg plan_uuid and sync_id. Use it to make sure we get the right vehicle route.
    for (const auto& receivedWp : receivedWptsList.items) {
        WaypointDTO wpToAdd;
        wpToAdd.getTaskUUID()           = QString::fromUtf8(receivedWp.task_uuid.data(), receivedWp.task_uuid.size());
        wpToAdd.getAutoContinue()       = receivedWp.autocontinue == 1 ? true : false;
        wpToAdd.getPosition().getX()    = receivedWp.x;
        wpToAdd.getPosition().getY()    = receivedWp.y;
        wpToAdd.getPosition().getZ()    = receivedWp.z;
        wpToAdd.getOrientation().getW() = receivedWp.param1;
        wpToAdd.getOrientation().getX() = receivedWp.param2;
        wpToAdd.getOrientation().getY() = receivedWp.param3;
        wpToAdd.getOrientation().getZ() = receivedWp.param4;

        if (receivedWp.command == MAV_CMD::MAV_CMD_NAV_INSP_POINT
            || receivedWp.command == MAV_CMD::MAV_CMD_NAV_INSP_AREA) {
            wpToAdd.getWpType() = WaypointType::ACTION;
        } else if (receivedWp.command == MAV_CMD::MAV_CMD_NAV_WAYPOINT_QUATERNION) {
            wpToAdd.getWpType() = WaypointType::POSE;
        }

        receivedRoute.getWpsList().append(wpToAdd);
    }

    return receivedRoute;
}

} // namespace gcs
