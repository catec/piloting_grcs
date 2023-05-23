#pragma once

#include <QMap>

#include "BasicMissionDTO.h"
#include "HomeDTO.h"
#include "dataModel/RouteDTO.h"

namespace gcs {

class MissionDTO : public IDTO
{
  public:
    virtual ~MissionDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::MISSION; }

    quint16& getSeqNRoute() Q_DECL_NOEXCEPT { return _seqNRoute; }

    const quint16& getSeqNRoute() const Q_DECL_NOEXCEPT { return _seqNRoute; }

    quint16& getSeqNWaypoint() Q_DECL_NOEXCEPT { return _seqNWaypoint; }

    const quint16& getSeqNWaypoint() const Q_DECL_NOEXCEPT { return _seqNWaypoint; }

    BasicMissionDTO& getBasicMission() Q_DECL_NOEXCEPT { return _basicMission; }

    const BasicMissionDTO& getBasicMission() const Q_DECL_NOEXCEPT { return _basicMission; }

    QMap<int, RouteDTO>& getLocalRoutesMap() Q_DECL_NOEXCEPT { return _localRoutesMap; }

    const QMap<int, RouteDTO>& getLocalRoutesMap() const Q_DECL_NOEXCEPT { return _localRoutesMap; }

    quint16& getCurrentRouteId() Q_DECL_NOEXCEPT { return _currentRouteId; }

    const quint16& getCurrentRouteId() const Q_DECL_NOEXCEPT { return _currentRouteId; }

    quint16& getCurrentWaypointId() Q_DECL_NOEXCEPT { return _currentWaypointId; }

    const quint16& getCurrentWaypointId() const Q_DECL_NOEXCEPT { return _currentWaypointId; }

    HomeDTO& getLoadedHome() Q_DECL_NOEXCEPT { return _loadedHome; }

    const HomeDTO& getLoadedHome() const Q_DECL_NOEXCEPT { return _loadedHome; }

    RouteDTO& getLoadedRoute() Q_DECL_NOEXCEPT { return _loadedRoute; }

    const RouteDTO& getLoadedRoute() const Q_DECL_NOEXCEPT { return _loadedRoute; }

    /// Operators
    MissionDTO& operator=(const MissionDTO& o) Q_DECL_NOEXCEPT
    {
        _seqNRoute         = o._seqNRoute;
        _seqNWaypoint      = o._seqNWaypoint;
        _basicMission      = o._basicMission;
        _localRoutesMap    = o._localRoutesMap;
        _currentRouteId    = o._currentRouteId;
        _currentWaypointId = o._currentWaypointId;
        _loadedHome        = o._loadedHome;
        _loadedRoute       = o._loadedRoute;

        return *this;
    }

    bool operator==(const MissionDTO& o) const Q_DECL_NOEXCEPT
    {
        return _seqNRoute == o._seqNRoute && _seqNWaypoint == o._seqNWaypoint && _basicMission == o._basicMission
            && _localRoutesMap == o._localRoutesMap && _currentRouteId == o._currentRouteId
            && _currentWaypointId == o._currentWaypointId && _loadedHome == o._loadedHome
            && _loadedRoute == o._loadedRoute;
    }

    bool operator!=(const MissionDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    quint16             _seqNRoute{1};
    quint16             _seqNWaypoint{1};
    BasicMissionDTO     _basicMission;
    QMap<int, RouteDTO> _localRoutesMap;
    quint16             _currentRouteId{0};
    quint16             _currentWaypointId{0};
    HomeDTO             _loadedHome;
    RouteDTO            _loadedRoute;
};

} // namespace gcs
