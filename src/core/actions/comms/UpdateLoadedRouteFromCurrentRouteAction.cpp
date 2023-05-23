
#include "UpdateLoadedRouteFromCurrentRouteAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/DataModel.h>

namespace gcs {
UpdateLoadedRouteFromCurrentRouteAction::UpdateLoadedRouteFromCurrentRouteAction(
        DataModel& dataModel, QObject* parent) :
        BasicAction(parent),
        _dataModel(dataModel),
        _routeBusiness(dataModel.getMissionDTO()),
        _currentMissionResultBusiness(dataModel.getCurrentMissionResultDTO())
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

UpdateLoadedRouteFromCurrentRouteAction::~UpdateLoadedRouteFromCurrentRouteAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType UpdateLoadedRouteFromCurrentRouteAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::UPDATE_LOADED_ROUTE_FROM_CURRENT_ROUTE;
}

void UpdateLoadedRouteFromCurrentRouteAction::execute(QSharedPointer<IDTO>)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        if (_dataModel.getMissionStatusDTO().getStatus() == MissionStatusType::NoMission) {
            throw std::runtime_error("No mission open");
        }

        _routeBusiness.editLoadedRouteWithLocal();
        _currentMissionResultBusiness.saveLoadedRouteToFile(_dataModel.getMissionDTO().getLoadedRoute());
        _currentMissionResultBusiness.createMissionTaskListFromRoute(
                _dataModel.getMissionDTO().getLoadedRoute(), _dataModel.getInspectionPlanDTO().getInspectionTaskList());
        _currentMissionResultBusiness.saveMissionResultToJson();

        auto missionDTO = createSharedDTO<MissionDTO>();
        *missionDTO     = _dataModel.getMissionDTO();
        Q_EMIT sendDTO(missionDTO);

        auto currentMissionResultDTO = createSharedDTO<CurrentMissionResultDTO>();
        *currentMissionResultDTO     = _dataModel.getCurrentMissionResultDTO();
        Q_EMIT sendDTO(currentMissionResultDTO);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << " - Error:" << e.what();

        sendWarningMsg(tr("Update loaded route from current error - ") + tr(e.what()));
    }
}

} // namespace gcs
