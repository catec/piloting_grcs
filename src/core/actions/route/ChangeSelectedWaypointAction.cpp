#include "ChangeSelectedWaypointAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/DataModel.h>

namespace gcs {

ChangeSelectedWaypointAction::ChangeSelectedWaypointAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent), _dataModel(dataModel), _routeBusiness(dataModel.getMissionDTO())
{
    QLOG_TRACE() << "ChangeSelectedWaypointAction::ChangeSelectedWaypointAction()";
}

ChangeSelectedWaypointAction::~ChangeSelectedWaypointAction()
{
    QLOG_TRACE() << "ChangeSelectedWaypointAction::~ChangeSelectedWaypointAction()";
}

ActionType ChangeSelectedWaypointAction::getActionType()
{
    QLOG_TRACE() << "ChangeSelectedWaypointAction::getActionType()";

    return ActionType::CHANGE_SELECTED_WAYPOINT;
}

void ChangeSelectedWaypointAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << "ChangeSelectedWaypointAction::execute()";

    try {
        if (_dataModel.getMissionStatusDTO().getStatus() == MissionStatusType::NoMission) {
            throw std::runtime_error("No mission open");
        }

        auto parsedDTO = dto.dynamicCast<BasicWaypointDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be BasicWaypointDTO");
        }

        _routeBusiness.changeSelectedWaypoint(parsedDTO->getId());

        auto missionDTO = createSharedDTO<MissionDTO>();
        *missionDTO     = _dataModel.getMissionDTO();
        Q_EMIT sendDTO(missionDTO);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << "ChangeSelectedWaypointAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Change selected waypoint error - ") + tr(e.what()));
    }
}

} // namespace gcs
