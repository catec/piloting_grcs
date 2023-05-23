#include "RemoveWaypointAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/DataModel.h>

namespace gcs {

RemoveWaypointAction::RemoveWaypointAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent),
        _dataModel(dataModel),
        _routeBusiness(dataModel.getMissionDTO()),
        _missionStatusBusiness(dataModel.getMissionStatusDTO()),
        _appStatusBusiness(dataModel.getApplicationStatusDTO())
{
    QLOG_TRACE() << "RemoveWaypointAction::RemoveWaypointAction()";
}

RemoveWaypointAction::~RemoveWaypointAction()
{
    QLOG_TRACE() << "RemoveWaypointAction::~RemoveWaypointAction()";
}

ActionType RemoveWaypointAction::getActionType()
{
    QLOG_TRACE() << "RemoveWaypointAction::getActionType()";

    return ActionType::REMOVE_WAYPOINT;
}

void RemoveWaypointAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << "RemoveWaypointAction::execute()";

    try {
        if (_dataModel.getMissionStatusDTO().getStatus() == MissionStatusType::NoMission) {
            throw std::runtime_error("No mission open");
        }

        auto parsedDTO = dto.dynamicCast<BasicWaypointDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be BasicWaypointDTO");
        }

        const int prevRoutesNum = _dataModel.getMissionDTO().getLocalRoutesMap().size();
        _routeBusiness.removeWaypoint(parsedDTO->getId());
        _missionStatusBusiness.changeStatus(MissionStatusType::WithChanges);
        if (_dataModel.getMissionDTO().getLocalRoutesMap().size() != prevRoutesNum) {
            _appStatusBusiness.changeStatus(AppStatusType::CreatingRoute);
        }

        auto missionStatusDTO = createSharedDTO<MissionStatusDTO>();
        *missionStatusDTO     = _dataModel.getMissionStatusDTO();
        Q_EMIT sendDTO(missionStatusDTO);

        auto appStatusDTO = createSharedDTO<ApplicationStatusDTO>();
        *appStatusDTO     = _dataModel.getApplicationStatusDTO();
        Q_EMIT sendDTO(appStatusDTO);

        auto missionDTO = createSharedDTO<MissionDTO>();
        *missionDTO     = _dataModel.getMissionDTO();
        Q_EMIT sendDTO(missionDTO);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << "RemoveWaypointAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Remove waypoint error - ") + tr(e.what()));
    }
}

} // namespace gcs
