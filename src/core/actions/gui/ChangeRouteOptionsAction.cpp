#include "ChangeRouteOptionsAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/DataModel.h>

namespace gcs {

ChangeRouteOptionsAction::ChangeRouteOptionsAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent), _dataModel(dataModel), _routeOptionsBusiness(dataModel.getRouteOptionsDTO())
{
    QLOG_TRACE() << "ChangeRouteOptionsAction::ChangeRouteOptionsAction()";
}

ChangeRouteOptionsAction::~ChangeRouteOptionsAction()
{
    QLOG_TRACE() << "ChangeRouteOptionsAction::~ChangeRouteOptionsAction()";
}

ActionType ChangeRouteOptionsAction::getActionType()
{
    QLOG_TRACE() << "ChangeRouteOptionsAction::getActionType()";

    return ActionType::CHANGE_ROUTE_OPTIONS;
}

void ChangeRouteOptionsAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << "ChangeRouteOptionsAction::execute()";

    try {
        auto parsedDTO = dto.dynamicCast<RouteOptionsDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be RouteOptionsDTO");
        }

        _routeOptionsBusiness.changeRouteOptions(parsedDTO->getFlag());

        auto routeOptionsDTO = createSharedDTO<RouteOptionsDTO>();
        *routeOptionsDTO     = _dataModel.getRouteOptionsDTO();
        Q_EMIT sendDTO(routeOptionsDTO);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << "ChangeRouteOptionsAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Change route options error - ") + tr(e.what()));
    }
}

} // namespace gcs
