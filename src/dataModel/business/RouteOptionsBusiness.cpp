#include "RouteOptionsBusiness.h"

#include <QsLog/QsLog.h>

#include "../dtos/RouteOptionsDTO.h"

namespace gcs {

RouteOptionsBusiness::RouteOptionsBusiness(RouteOptionsDTO& routeOptionsDTO) : _routeOptionsDTO(routeOptionsDTO)
{
    QLOG_TRACE() << "RouteOptionsBusiness::RouteOptionsBusiness()";
}

RouteOptionsBusiness::~RouteOptionsBusiness()
{
    QLOG_TRACE() << "RouteOptionsBusiness::~RouteOptionsBusiness()";
}

void RouteOptionsBusiness::changeRouteOptions(const RouteOptions& flag) Q_DECL_NOEXCEPT
{
    QLOG_TRACE() << "RouteOptionsBusiness::changeRouteOptions()";

    QLOG_DEBUG() << "MissionStatusBusiness::changeStatus() - "
                    "Route Options Flag:"
                 << flag;

    _routeOptionsDTO.getFlag() = flag;
}

} // namespace gcs
