#pragma once

#include <core/dtos/BasicRouteDTO.h>
#include <dataModel/WaypointDTO.h>

#include <QList>

namespace gcs {

class RouteDTO : public BasicRouteDTO
{
  public:
    virtual ~RouteDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::ROUTE; }

    QList<WaypointDTO>& getWpsList() Q_DECL_NOEXCEPT { return _wpsList; }

    const QList<WaypointDTO>& getWpsList() const Q_DECL_NOEXCEPT { return _wpsList; }

    /// Operators
    RouteDTO& operator=(const RouteDTO& o) Q_DECL_NOEXCEPT
    {
        _wpsList = o._wpsList;

        return *this;
    }

    bool operator==(const RouteDTO& o) const Q_DECL_NOEXCEPT { return _wpsList == o._wpsList; }

    bool operator!=(const RouteDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    QList<WaypointDTO> _wpsList;
};

} // namespace gcs
