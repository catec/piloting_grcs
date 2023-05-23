#include "AddRouteAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/DataModel.h>

namespace gcs {

AddRouteAction::AddRouteAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent),
        _dataModel(dataModel),
        _routeBusiness(dataModel.getMissionDTO()),
        _missionStatusBusiness(dataModel.getMissionStatusDTO()),
        _appStatusBusiness(dataModel.getApplicationStatusDTO())
{
    QLOG_TRACE() << "AddRouteAction::AddRouteAction()";
}

AddRouteAction::~AddRouteAction()
{
    QLOG_TRACE() << "AddRouteAction::~AddRouteAction()";
}

ActionType AddRouteAction::getActionType()
{
    QLOG_TRACE() << "AddRouteAction::getActionType()";

    return ActionType::ADD_ROUTE;
}

void AddRouteAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << "AddRouteAction::execute()";

    try {
        if (_dataModel.getMissionStatusDTO().getStatus() == MissionStatusType::NoMission) {
            throw std::runtime_error("No mission open");
        }

        auto parsedDTO = dto.dynamicCast<WaypointDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be WaypointDTO");
        }

        _routeBusiness.addRoute(*parsedDTO);
        _missionStatusBusiness.changeStatus(MissionStatusType::WithChanges);
        _appStatusBusiness.changeStatus(AppStatusType::EditingRoute);

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
        QLOG_WARN() << "AddRouteAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Add route error - ") + tr(e.what()));
    }
}

} // namespace gcs
