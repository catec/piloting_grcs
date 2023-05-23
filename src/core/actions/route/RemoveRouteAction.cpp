#include "RemoveRouteAction.h"

#include <QsLog/QsLog.h>
#include <core/dtos/BasicRouteDTO.h>
#include <dataModel/DataModel.h>

namespace gcs {

RemoveRouteAction::RemoveRouteAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent),
        _dataModel(dataModel),
        _routeBusiness(dataModel.getMissionDTO()),
        _missionStatusBusiness(dataModel.getMissionStatusDTO()),
        _appStatusBusiness(dataModel.getApplicationStatusDTO())
{
    QLOG_TRACE() << "RemoveRouteAction::RemoveRouteAction()";
}

RemoveRouteAction::~RemoveRouteAction()
{
    QLOG_TRACE() << "RemoveRouteAction::~RemoveRouteAction()";
}

ActionType RemoveRouteAction::getActionType()
{
    QLOG_TRACE() << "RemoveRouteAction::getActionType()";

    return ActionType::REMOVE_ROUTE;
}

void RemoveRouteAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << "RemoveRouteAction::execute()";

    try {
        if (_dataModel.getMissionStatusDTO().getStatus() == MissionStatusType::NoMission) {
            throw std::runtime_error("No mission open");
        }

        auto parsedDTO = dto.dynamicCast<BasicRouteDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be BasicRouteDTO");
        }

        _routeBusiness.removeRoute(parsedDTO->getId());
        _missionStatusBusiness.changeStatus(MissionStatusType::WithChanges);
        _appStatusBusiness.changeStatus(AppStatusType::CreatingRoute);

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
        QLOG_WARN() << "RemoveRouteAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Remove route error - ") + tr(e.what()));
    }
}

} // namespace gcs
