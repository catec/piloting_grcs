
#include "RemoveLoadedRouteAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/DataModel.h>

namespace gcs {
RemoveLoadedRouteAction::RemoveLoadedRouteAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent), _dataModel(dataModel), _routeBusiness(dataModel.getMissionDTO())
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

RemoveLoadedRouteAction::~RemoveLoadedRouteAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType RemoveLoadedRouteAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::REMOVE_LOADED_ROUTE;
}

void RemoveLoadedRouteAction::execute(QSharedPointer<IDTO>)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        _routeBusiness.removeLoadedRoute();

        auto missionDTO = createSharedDTO<MissionDTO>();
        *missionDTO     = _dataModel.getMissionDTO();
        Q_EMIT sendDTO(missionDTO);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << " - Error:" << e.what();

        sendWarningMsg(tr("Remove loaded route error - ") + tr(e.what()));
    }
}
} // namespace gcs
