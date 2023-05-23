
#ifndef MAVSDK_COMMS_H_
#define MAVSDK_COMMS_H_

#include <communications/ICommunications.h>
#include <core/IAction.h>
#include <mavsdk/plugins/alarm/alarm.h>
#include <mavsdk/plugins/checklist/checklist.h>
#include <mavsdk/plugins/command/command.h>
#include <mavsdk/plugins/hl_action/hl_action.h>
#include <mavsdk/plugins/inspection/inspection.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <QObject>
#include <memory>

#include "gcs_communications_export.h"

namespace mavsdk {
class Mavsdk;
class Telemetry;
class Alarm;
class Inspection;
class Checklist;
class HLAction;
class Command;
} // namespace mavsdk

namespace gcs {
class CommsLinkDTO;
class CommsLinkMavsdkDTO;
class MissionDTO;
class RouteDTO;
class CheckListDTO;
class HLActionListDTO;
class AlarmListDTO;
class AlarmStatusDTO;
class HomeDTO;
class ParamDTO;
class ActionParametersDTO;

class PILOTING_GRCS_COMMUNICATIONS_EXPORT MavsdkComms : public QObject, public ICommunications
{
    Q_OBJECT

  public:
    explicit MavsdkComms(QObject* parent = nullptr);
    ~MavsdkComms();

    bool isConnected() const override;

  public Q_SLOTS:
    void connectTo(QSharedPointer<CommsLinkDTO>) override;
    void disconnectFrom() override;

    void connectToRoboticSystem(const CommsLinkMavsdkDTO&);

    void downloadWaypointsList() override;
    void sendWaypointsList(const QString&, const quint32&, const MissionDTO&, const QList<InspectionTaskDTO>&) override;
    void setCurrentWaypointItem(const quint16) override;

    void downloadCheckList() override;
    void downloadAlarmsList() override;
    void downloadHighLevelActions() override;

    void sendCommand(QSharedPointer<CommandLongDTO>) override;

    void sendHome(QSharedPointer<HomeDTO>);
    void startProgressBar(const QString&);
    void finishProgressBar();

  private:
    void sendConsoleMsg(const MsgType&, const QString&);
    void sendPopupMsg(const QString&, const MsgType&, const QString&);

    void checkIfHeartBeatTimedOut();
    void resetMavsdkModules();

    void configureTelemetryCallbacks();
    void configureAlarmCallbacks();
    void configureInspectionCallbacks();

    void heartbeatCb(uint64_t);
    void uploadInspectionCb(mavsdk::InspectionBase::Result, mavsdk::InspectionBase::Ack);
    void downloadInspectionCb(mavsdk::InspectionBase::Result, mavsdk::InspectionBase::WaypointList);
    void downloadCheckListCb(mavsdk::ChecklistBase::Result, mavsdk::ChecklistBase::Checklist);
    void downloadAlarmListCb(mavsdk::AlarmBase::Result, mavsdk::AlarmBase::AlarmList);
    void downloadHighLevelActionsCb(mavsdk::HLActionBase::Result, mavsdk::HLActionBase::HLActionList);
    void sendCommandCb(mavsdk::CommandBase::Result, mavsdk::CommandBase::CommandAck);

    void validateRoute(const MissionDTO&);
    bool areEqualRoutes(const RouteDTO&, const RouteDTO&);

    bool showConfirmationDialog(const QString&, const QString&);

    mavsdk::InspectionBase::WaypointList convertRouteToWaypointList(
            const QString&, const quint32& syncId, const RouteDTO&, const QList<InspectionTaskDTO>&);
    void convertInspectionAreaToWaypoints(
            const InspectionTaskDTO&, const bool&, const ActionParametersDTO&, mavsdk::InspectionBase::WaypointList&);
    void convertInspectionPointToWaypoints(
            const InspectionTaskDTO&, const bool&, const ActionParametersDTO&, mavsdk::InspectionBase::WaypointList&);
    float getParamValueFromIndex(const qint16&, const QList<ParamDTO>&);

    mavsdk::CommandBase::CommandLong convertFromDTO(const CommandLongDTO&);

    CheckListDTO    convertToDTO(const mavsdk::ChecklistBase::Checklist&);
    HLActionListDTO convertToDTO(const mavsdk::HLActionBase::HLActionList&);
    AlarmListDTO    convertToDTO(const mavsdk::AlarmBase::AlarmList&);
    AlarmStatusDTO  convertToDTO(const mavsdk::AlarmBase::AlarmStatus&);
    RouteDTO        convertToDTO(const mavsdk::InspectionBase::WaypointList&);
    QList<ParamDTO> convertToParamListDTO(const HomeDTO&);

  Q_SIGNALS:
    void sendDTOToCore(ActionType actionType, QSharedPointer<IDTO> dto = nullptr);

  private:
    std::unique_ptr<mavsdk::Mavsdk>     _mavsdk;
    std::unique_ptr<mavsdk::Telemetry>  _mavsdkTelemetry;
    std::unique_ptr<mavsdk::Alarm>      _mavsdkAlarm;
    std::unique_ptr<mavsdk::Inspection> _mavsdkInspection;
    std::unique_ptr<mavsdk::Checklist>  _mavsdkChecklist;
    std::unique_ptr<mavsdk::HLAction>   _mavsdkHLAction;
    std::unique_ptr<mavsdk::Command>    _mavsdkCommand;

    bool _heartbeat_timeout;
};
} // namespace gcs

// #include "MavsdkComms.inl"

#endif
