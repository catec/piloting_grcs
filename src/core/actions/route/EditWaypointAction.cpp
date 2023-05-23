#include "EditWaypointAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/DataModel.h>

namespace gcs {

EditWaypointAction::EditWaypointAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent),
        _dataModel(dataModel),
        _routeBusiness(dataModel.getMissionDTO()),
        _missionStatusBusiness(dataModel.getMissionStatusDTO())
{
    QLOG_TRACE() << "EditWaypointAction::EditWaypointAction()";
}

EditWaypointAction::~EditWaypointAction()
{
    QLOG_TRACE() << "EditWaypointAction::~EditWaypointAction()";
}

ActionType EditWaypointAction::getActionType()
{
    QLOG_TRACE() << "EditWaypointAction::getActionType()";

    return ActionType::EDIT_WAYPOINT;
}

void EditWaypointAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << "EditWaypointAction::execute()";

    try {
        if (_dataModel.getMissionStatusDTO().getStatus() == MissionStatusType::NoMission) {
            throw std::runtime_error("No mission open");
        }

        auto parsedDTO = dto.dynamicCast<WaypointDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be WaypointDTO");
        }

        _routeBusiness.editWaypoint(*parsedDTO);
        _missionStatusBusiness.changeStatus(MissionStatusType::WithChanges);

        auto missionStatusDTO = createSharedDTO<MissionStatusDTO>();
        *missionStatusDTO     = _dataModel.getMissionStatusDTO();
        Q_EMIT sendDTO(missionStatusDTO);

        auto missionDTO = createSharedDTO<MissionDTO>();
        *missionDTO     = _dataModel.getMissionDTO();
        Q_EMIT sendDTO(missionDTO);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << "EditWaypointAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Edit waypoint error - ") + tr(e.what()));
    }
}

} // namespace gcs
