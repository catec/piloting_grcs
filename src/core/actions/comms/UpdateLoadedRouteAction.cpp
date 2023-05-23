
#include "UpdateLoadedRouteAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/DataModel.h>

namespace gcs {
UpdateLoadedRouteAction::UpdateLoadedRouteAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent),
        _dataModel(dataModel),
        _routeBusiness(dataModel.getMissionDTO()),
        _currentMissionResultBusiness(dataModel.getCurrentMissionResultDTO())
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

UpdateLoadedRouteAction::~UpdateLoadedRouteAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType UpdateLoadedRouteAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::UPDATE_LOADED_ROUTE;
}

void UpdateLoadedRouteAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        auto parsedDTO = dto.dynamicCast<RouteDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be RouteDTO");
        }

        _routeBusiness.editLoadedRoute(*parsedDTO);
        _currentMissionResultBusiness.saveLoadedRouteToFile(*parsedDTO);
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

        sendWarningMsg(tr("Update loaded route error - ") + tr(e.what()));
    }
}

} // namespace gcs
