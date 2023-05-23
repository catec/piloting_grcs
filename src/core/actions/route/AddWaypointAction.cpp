#include "AddWaypointAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/DataModel.h>
#include <dataModel/PoseDTO.h>

namespace gcs {

AddWaypointAction::AddWaypointAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent),
        _dataModel(dataModel),
        _routeBusiness(dataModel.getMissionDTO()),
        _missionStatusBusiness(dataModel.getMissionStatusDTO())
{
    QLOG_TRACE() << "AddWaypointAction::AddWaypointAction()";
}

AddWaypointAction::~AddWaypointAction()
{
    QLOG_TRACE() << "AddWaypointAction::~AddWaypointAction()";
}

ActionType AddWaypointAction::getActionType()
{
    QLOG_TRACE() << "AddWaypointAction::getActionType()";

    return ActionType::ADD_WAYPOINT;
}

void AddWaypointAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << "AddWaypointAction::execute()";

    try {
        if (_dataModel.getMissionStatusDTO().getStatus() == MissionStatusType::NoMission) {
            throw std::runtime_error("No mission open");
        }

        auto parsedDTO   = dto.dynamicCast<PoseDTO>();
        auto waypointDTO = dto.dynamicCast<WaypointDTO>();
        if (parsedDTO) {
            _routeBusiness.addWaypoint(*parsedDTO, _dataModel.getAutoHeadingMode());
        } else if (waypointDTO) {
            _routeBusiness.addWaypoint(*waypointDTO, _dataModel.getAutoHeadingMode());
        } else {
            throw std::runtime_error("DTO type should be WaypointDTO or PoseDTO");
        }

        _missionStatusBusiness.changeStatus(MissionStatusType::WithChanges);

        auto missionStatusDTO = createSharedDTO<MissionStatusDTO>();
        *missionStatusDTO     = _dataModel.getMissionStatusDTO();
        Q_EMIT sendDTO(missionStatusDTO);

        auto missionDTO = createSharedDTO<MissionDTO>();
        *missionDTO     = _dataModel.getMissionDTO();
        Q_EMIT sendDTO(missionDTO);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << "AddWaypointAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Add waypoint error - ") + tr(e.what()));
    }
}

} // namespace gcs
