#pragma once

#include <QPoint>

#include "dataModel/WaypointDTO.h"

namespace gcs {
class Waypoint3dItem
{
  public:
    Waypoint3dItem(){};
    ~Waypoint3dItem(){};

    quint16&       id() Q_DECL_NOEXCEPT { return _waypointId; }
    const quint16& id() const Q_DECL_NOEXCEPT { return _waypointId; }

    quint16&       seqNumber() Q_DECL_NOEXCEPT { return _waypointSeqId; }
    const quint16& seqNumber() const Q_DECL_NOEXCEPT { return _waypointSeqId; }

    quint16&       routeId() Q_DECL_NOEXCEPT { return _routeId; }
    const quint16& routeId() const Q_DECL_NOEXCEPT { return _routeId; }

    WaypointDTO&       waypoint() Q_DECL_NOEXCEPT { return _waypoint; }
    const WaypointDTO& waypoint() const Q_DECL_NOEXCEPT { return _waypoint; }

    bool&       inLoadedRoute() Q_DECL_NOEXCEPT { return _inLoadedRoute; }
    const bool& inLoadedRoute() const Q_DECL_NOEXCEPT { return _inLoadedRoute; }

    bool&       inSelectedRoute() Q_DECL_NOEXCEPT { return _inSelectedRoute; }
    const bool& inSelectedRoute() const Q_DECL_NOEXCEPT { return _inSelectedRoute; }

    bool&           isSelectedWaypoint() Q_DECL_NOEXCEPT { return _isSelectedWaypoint; }
    const bool&     isSelectedWaypoint() const Q_DECL_NOEXCEPT { return _isSelectedWaypoint; }
    /// Operators
    Waypoint3dItem& operator=(const Waypoint3dItem& o) Q_DECL_NOEXCEPT
    {
        _waypointId         = o._waypointId;
        _waypointSeqId      = o._waypointSeqId;
        _routeId            = o._routeId;
        _waypoint           = o._waypoint;
        _inLoadedRoute      = o._inLoadedRoute;
        _isSelectedWaypoint = o._isSelectedWaypoint;
        _inSelectedRoute    = o._inSelectedRoute;
        return *this;
    }

    bool operator==(const Waypoint3dItem& o) const Q_DECL_NOEXCEPT
    {
        return _waypointId == o._waypointId && _waypointSeqId == o._waypointSeqId && _routeId == o._routeId
            && _waypoint == o._waypoint && _inLoadedRoute == o._inLoadedRoute
            && _isSelectedWaypoint == o._isSelectedWaypoint && _inSelectedRoute == o._inSelectedRoute;
    }

    bool operator!=(const Waypoint3dItem& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    quint16 _waypointId{0};
    quint16 _waypointSeqId{0};
    quint16 _routeId{0};

    WaypointDTO _waypoint;

    bool _inLoadedRoute{false};
    bool _inSelectedRoute{false};
    bool _isSelectedWaypoint{false};
};
} // namespace gcs
