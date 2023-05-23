#pragma once

#include <common/Types.h>

#include <QtGlobal>

#include "gcs_dataModel_export.h"

namespace gcs {

class RouteOptionsDTO;

class PILOTING_GRCS_DATAMODEL_EXPORT RouteOptionsBusiness
{
  public:
    explicit RouteOptionsBusiness(RouteOptionsDTO& routeOptionsDTO);
    virtual ~RouteOptionsBusiness();

    void changeRouteOptions(const RouteOptions&) Q_DECL_NOEXCEPT;

  private:
    RouteOptionsDTO& _routeOptionsDTO;
};

} // namespace gcs
