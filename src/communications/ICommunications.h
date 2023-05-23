#pragma once

#include <communications/MavSDK/dtos/CommandLongDTO.h>
#include <core/IAction.h>

#include <memory>

#include "gcs_communications_export.h"

namespace gcs {

class CommsLinkDTO;
class MissionDTO;
class InspectionTaskDTO;
class HomeParametersDTO;
class HomeDTO;
class MessageIntervalDTO;

/*!
 * @brief Interface of the communications layer running in the GCS.
 *        All protocols must implement it.
 */
class PILOTING_GRCS_COMMUNICATIONS_EXPORT ICommunications
{
  public:
    template <typename _Type, typename... _T>
    static std::unique_ptr<ICommunications> create(_T&&... _arg)
    {
        return std::make_unique<_Type>(std::forward<_T>(_arg)...);
    }

    virtual ~ICommunications() {}

    virtual bool isConnected() const = 0;

    /// public Q_SLOTS:
    /// @brief Connect to robotics system based on CommsLinkDTO
    virtual void connectTo(QSharedPointer<CommsLinkDTO>) = 0;

    /// @brief Disconnect from robotics system
    virtual void disconnectFrom() = 0;

    /// @brief Send waypoint list with mission tasks to robotic system
    virtual void sendWaypointsList(
            const QString&, const quint32&, const MissionDTO& mission, const QList<InspectionTaskDTO>&)
            = 0;

    /// @brief Receive waypoint list that there is loaded on robotics system
    virtual void downloadWaypointsList() = 0;

    /// @brief Change the current waypoint item of the robotic vehicle
    virtual void setCurrentWaypointItem(const quint16) = 0;

    /// @brief Download checklist from robotic system
    virtual void downloadCheckList() = 0;

    /// @brief Download alarm list from robotic system
    virtual void downloadAlarmsList() = 0;

    /// @brief Download high level actions list from robotic system
    virtual void downloadHighLevelActions() = 0;

    /// @brief Send command to robotic vehicle
    virtual void sendCommand(QSharedPointer<CommandLongDTO>) = 0;

    /// Q_SIGNALS:
    virtual void sendDTOToCore(ActionType actionType, QSharedPointer<IDTO> dto = nullptr) = 0;
};

} // namespace gcs
