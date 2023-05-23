#include "CopyLoadedRouteToLocalAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/DataModel.h>

namespace gcs {

CopyLoadedRouteToLocalAction::CopyLoadedRouteToLocalAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent),
        _dataModel(dataModel),
        _routeBusiness(dataModel.getMissionDTO()),
        _missionStatusBusiness(dataModel.getMissionStatusDTO()),
        _appStatusBusiness(dataModel.getApplicationStatusDTO())
{
    QLOG_TRACE() << "CopyLoadedRouteToLocalAction::CopyLoadedRouteToLocalAction()";
}

CopyLoadedRouteToLocalAction::~CopyLoadedRouteToLocalAction()
{
    QLOG_TRACE() << "CopyLoadedRouteToLocalAction::~CopyLoadedRouteToLocalAction()";
}

ActionType CopyLoadedRouteToLocalAction::getActionType()
{
    QLOG_TRACE() << "CopyLoadedRouteToLocalAction::getActionType()";

    return ActionType::COPY_LOADED_ROUTE_TO_LOCAL;
}

void CopyLoadedRouteToLocalAction::execute(QSharedPointer<IDTO>)
{
    QLOG_TRACE() << "CopyLoadedRouteToLocalAction::execute()";

    try {
        if (_dataModel.getMissionStatusDTO().getStatus() == MissionStatusType::NoMission) {
            throw std::runtime_error("No mission open");
        }

        _routeBusiness.copyLoadedRouteToLocal();
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
        QLOG_WARN() << "CopyLoadedRouteToLocalAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Copy loaded route to local error - ") + tr(e.what()));
    }
}

} // namespace gcs
