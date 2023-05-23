#pragma once

#include <dataModel/dtos/InspectionTaskDTO.h>
#include <dataModel/dtos/VisualizationConfigDTO.h>

#include <QMap>
#include <QPoint>

#include "common/IDTO.h"
#include "common/Types.h"
#include "dataModel/Position2dDTO.h"
#include "dataModel/dtos/HomeDTO.h"
#include "maps/items/Waypoint3dItem.h"

namespace gcs {
class Map3dDTO : public IDTO
{
  public:
    virtual ~Map3dDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::MAP_3D; }

    AppStatusType&       getAppStatus() Q_DECL_NOEXCEPT { return _appStatus; }
    const AppStatusType& getAppStatus() const Q_DECL_NOEXCEPT { return _appStatus; }

    ViewModeType&       getViewMode() Q_DECL_NOEXCEPT { return _viewMode; }
    const ViewModeType& getViewMode() const Q_DECL_NOEXCEPT { return _viewMode; }

    float&       getDefaultAltitude() Q_DECL_NOEXCEPT { return _defaultAltitude; }
    const float& getDefaultAltitude() const Q_DECL_NOEXCEPT { return _defaultAltitude; }

    bool&       getCommsIsConnected() Q_DECL_NOEXCEPT { return _commsIsConnected; }
    const bool& getCommsIsConnected() const Q_DECL_NOEXCEPT { return _commsIsConnected; }

    QList<QSharedPointer<Waypoint3dItem>>&       getWaypointsList() Q_DECL_NOEXCEPT { return _waypointsList; }
    const QList<QSharedPointer<Waypoint3dItem>>& getWaypointsList() const Q_DECL_NOEXCEPT { return _waypointsList; }

    QList<QSharedPointer<Waypoint3dItem>>& getVisibleWaypointsList() Q_DECL_NOEXCEPT { return _visibleWaypointList; }
    const QList<QSharedPointer<Waypoint3dItem>>& getVisibleWaypointsList() const Q_DECL_NOEXCEPT
    {
        return _visibleWaypointList;
    }

    QList<Position3dDTO>&       getMeasureList() Q_DECL_NOEXCEPT { return _measureList; }
    const QList<Position3dDTO>& getMeasureList() const Q_DECL_NOEXCEPT { return _measureList; }

    HomeDTO&       getLoadedHome() Q_DECL_NOEXCEPT { return _loadedHome; }
    const HomeDTO& getLoadedHome() const Q_DECL_NOEXCEPT { return _loadedHome; }

    bool&       getInEditionMode() Q_DECL_NOEXCEPT { return _inEditionMode; }
    const bool& getInEditionMode() const Q_DECL_NOEXCEPT { return _inEditionMode; }

    qint16&       getCurrentWpIndex() Q_DECL_NOEXCEPT { return _currentWpIndex; }
    const qint16& getCurrentWpIndex() const Q_DECL_NOEXCEPT { return _currentWpIndex; }

    QList<InspectionTaskDTO>&       getInspectionTaskList() Q_DECL_NOEXCEPT { return _inspectionTaskList; }
    const QList<InspectionTaskDTO>& getInspectionTaskList() const Q_DECL_NOEXCEPT { return _inspectionTaskList; }

    VisualizationConfigDTO&       getVisualizationConfig() Q_DECL_NOEXCEPT { return _visualizationConfig; }
    const VisualizationConfigDTO& getVisualizationConfig() const Q_DECL_NOEXCEPT { return _visualizationConfig; }
    /// Operators
    Map3dDTO&                     operator=(const Map3dDTO& o) Q_DECL_NOEXCEPT
    {
        _appStatus           = o._appStatus;
        _viewMode            = o._viewMode;
        _defaultAltitude     = o._defaultAltitude;
        _commsIsConnected    = o._commsIsConnected;
        _waypointsList       = o._waypointsList;
        _visibleWaypointList = o._visibleWaypointList;
        _inEditionMode       = o._inEditionMode;
        _loadedHome          = o._loadedHome;
        _measureList         = o._measureList;
        _currentWpIndex      = o._currentWpIndex;
        _inspectionTaskList  = o._inspectionTaskList;
        _visualizationConfig = o._visualizationConfig;

        return *this;
    }

    bool operator==(const Map3dDTO& o) const Q_DECL_NOEXCEPT
    {
        return _appStatus == o._appStatus && _viewMode == o._viewMode && _defaultAltitude == o._defaultAltitude
            && _commsIsConnected == o._commsIsConnected && _waypointsList == o._waypointsList
            && _visibleWaypointList == o._visibleWaypointList && _inEditionMode == o._inEditionMode
            && _loadedHome == o._loadedHome && _currentWpIndex == o._currentWpIndex
            && _inspectionTaskList == o._inspectionTaskList && _measureList == o._measureList
            && _visualizationConfig == o._visualizationConfig;
    }

    bool operator!=(const Map3dDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    AppStatusType _appStatus{AppStatusType::Panning};
    ViewModeType  _viewMode{ViewModeType::ThreeD};
    bool          _inEditionMode;

    float                    _defaultAltitude{2.5};
    bool                     _commsIsConnected{false};
    qint16                   _currentWpIndex{-1};
    QList<InspectionTaskDTO> _inspectionTaskList;

    QList<QSharedPointer<Waypoint3dItem>> _waypointsList;
    QList<QSharedPointer<Waypoint3dItem>> _visibleWaypointList;
    QList<Position3dDTO>                  _measureList;
    HomeDTO                               _loadedHome;
    VisualizationConfigDTO                _visualizationConfig;
};
} // namespace gcs
