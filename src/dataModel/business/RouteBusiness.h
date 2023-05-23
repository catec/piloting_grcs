#pragma once

#include <QtGlobal>

#include "gcs_dataModel_export.h"

namespace gcs {

class MissionDTO;
class PoseDTO;
class WaypointDTO;
class RouteDTO;

class PILOTING_GRCS_DATAMODEL_EXPORT RouteBusiness
{
  public:
    explicit RouteBusiness(MissionDTO& missionDTO);
    virtual ~RouteBusiness();

    void addRoute(const WaypointDTO&) Q_DECL_NOEXCEPT;
    void addRoute(const RouteDTO&);
    void removeRoute(const quint16);

    void addWaypoint(const PoseDTO&, const bool& autoHeading = false);
    void addWaypoint(const WaypointDTO&, const bool& autoHeading = false);
    void removeWaypoint(const quint16);
    void editWaypoint(const WaypointDTO&);
    void changeSelectedWaypoint(const quint16);

    void editLoadedRouteWithLocal();
    void editLoadedRoute(const RouteDTO&);
    void removeLoadedRoute() Q_DECL_NOEXCEPT;
    void copyLoadedRouteToLocal();

    void updateSeqNumbers() Q_DECL_NOEXCEPT;
    void selectLastWaypoint();

    void manageRouteAutoHeading();

  private:
    void applyAutoHeading(const WaypointDTO& lastWp, WaypointDTO& secondLastWp);

  private:
    MissionDTO& _missionDTO;
};

} // namespace gcs
