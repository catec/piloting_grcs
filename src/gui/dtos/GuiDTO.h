#pragma once

#include <communications/MavSDK/dtos/CheckListDTO.h>
#include <dataModel/RobotAttitudeAngVelocityDTO.h>
#include <dataModel/RouteDTO.h>
#include <dataModel/dtos/CommsLinkDDHLDTO.h>
#include <dataModel/dtos/CommsLinkMavsdkDTO.h>
#include <dataModel/dtos/HomeDTO.h>
#include <dataModel/dtos/InspectionPlanDTO.h>
#include <dataModel/dtos/MissionResultsDTO.h>
#include <dataModel/dtos/VisualizationConfigDTO.h>

namespace gcs {

class GuiDTO : public IDTO
{
  public:
    virtual ~GuiDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::GUI; }

    QString& getMissionName() Q_DECL_NOEXCEPT { return _missionName; }

    const QString& getMissionName() const Q_DECL_NOEXCEPT { return _missionName; }

    QString& getMissionDescription() Q_DECL_NOEXCEPT { return _missionDescription; }

    const QString& getMissionDescription() const Q_DECL_NOEXCEPT { return _missionDescription; }

    QDateTime& getMissionCreationDate() Q_DECL_NOEXCEPT { return _missionCreationDate; }

    const QDateTime& getMissionCreationDate() const Q_DECL_NOEXCEPT { return _missionCreationDate; }

    QDateTime& getMissionLastUpdateDate() Q_DECL_NOEXCEPT { return _missionLastUpdateDate; }

    const QDateTime& getMissionLastUpdateDate() const Q_DECL_NOEXCEPT { return _missionLastUpdateDate; }

    bool& getHasRoutes() Q_DECL_NOEXCEPT { return _hasRoutes; }

    const bool& getHasRoutes() const Q_DECL_NOEXCEPT { return _hasRoutes; }

    HomeDTO& getLoadedHome() Q_DECL_NOEXCEPT { return _loadedHome; }

    const HomeDTO& getLoadedHome() const Q_DECL_NOEXCEPT { return _loadedHome; }

    AppStatusType& getAppStatus() Q_DECL_NOEXCEPT { return _appStatus; }

    const AppStatusType& getAppStatus() const Q_DECL_NOEXCEPT { return _appStatus; }

    ViewModeType& getViewMode() Q_DECL_NOEXCEPT { return _viewMode; }

    const ViewModeType& getViewMode() const Q_DECL_NOEXCEPT { return _viewMode; }

    MissionStatusType& getMissionStatus() Q_DECL_NOEXCEPT { return _missionStatus; }

    const MissionStatusType& getMissionStatus() const Q_DECL_NOEXCEPT { return _missionStatus; }

    RouteOptions& getRouteOptions() Q_DECL_NOEXCEPT { return _routeOptions; }

    const RouteOptions& getRouteOptions() const Q_DECL_NOEXCEPT { return _routeOptions; }

    RouteDTO& getLoadedRoute() Q_DECL_NOEXCEPT { return _loadedRoute; }

    const RouteDTO& getLoadedRoute() const Q_DECL_NOEXCEPT { return _loadedRoute; }

    bool& getCommsIsConnected() Q_DECL_NOEXCEPT { return _commsIsConnected; }

    const bool& getCommsIsConnected() const Q_DECL_NOEXCEPT { return _commsIsConnected; }

    CommsLinkMavsdkDTO& getCommsMavsdkOptions() Q_DECL_NOEXCEPT { return _mavsdkCommsOptions; }

    const CommsLinkMavsdkDTO& getCommsMavsdkOptions() const Q_DECL_NOEXCEPT { return _mavsdkCommsOptions; }

    CommsLinkDDHLDTO& getCommsDDHLOptions() Q_DECL_NOEXCEPT { return _ddhlCommsOptions; }

    const CommsLinkDDHLDTO& getCommsDDHLOptions() const Q_DECL_NOEXCEPT { return _ddhlCommsOptions; }

    QSharedPointer<MissionResultsDTO>& getMissionResults() Q_DECL_NOEXCEPT { return _missionResults; }

    const QSharedPointer<MissionResultsDTO>& getMissionResults() const Q_DECL_NOEXCEPT { return _missionResults; }

    CheckListDTO& getCheckList() Q_DECL_NOEXCEPT { return _checkList; }

    const CheckListDTO& getCheckList() const Q_DECL_NOEXCEPT { return _checkList; }

    InspectionPlanDTO& getInspectionPlan() Q_DECL_NOEXCEPT { return _inspectionPlan; }

    const InspectionPlanDTO& getInspectionPlan() const Q_DECL_NOEXCEPT { return _inspectionPlan; }

    VisualizationConfigDTO& getVisualizationConfig() Q_DECL_NOEXCEPT { return _visualizationConfig; }

    const VisualizationConfigDTO& getVisualizationConfig() const Q_DECL_NOEXCEPT { return _visualizationConfig; }

    bool& getDownloadAssetIsCanceled() Q_DECL_NOEXCEPT { return _downloadAssetIsCanceled; }

    const bool& getDownloadAssetIsCanceled() const Q_DECL_NOEXCEPT { return _downloadAssetIsCanceled; }

    QString& getTargetInspectionPlanUUID() Q_DECL_NOEXCEPT { return _targetInspectionPlanUUID; }

    const QString& getTargetInspectionPlanUUID() const Q_DECL_NOEXCEPT { return _targetInspectionPlanUUID; }

    bool& getAutoHeadingMode() Q_DECL_NOEXCEPT { return _autoHeadingMode; }

    const bool& getAutoHeadingMode() const Q_DECL_NOEXCEPT { return _autoHeadingMode; }

    /// Operators
    GuiDTO& operator=(const GuiDTO& o) Q_DECL_NOEXCEPT
    {
        _missionName           = o._missionName;
        _missionDescription    = o._missionDescription;
        _missionCreationDate   = o._missionCreationDate;
        _missionLastUpdateDate = o._missionLastUpdateDate;
        _hasRoutes             = o._hasRoutes;
        _loadedHome            = o._loadedHome;
        _appStatus             = o._appStatus;
        _viewMode              = o._viewMode;
        _missionStatus         = o._missionStatus;
        _routeOptions          = o._routeOptions;
        _loadedRoute           = o._loadedRoute;
        _commsIsConnected      = o._commsIsConnected;

        _mavsdkCommsOptions = o._mavsdkCommsOptions;
        _ddhlCommsOptions   = o._ddhlCommsOptions;
        _checkList          = o._checkList;
        _inspectionPlan     = o._inspectionPlan;
        _missionResults     = o._missionResults;

        _downloadAssetIsCanceled  = o._downloadAssetIsCanceled;
        _targetInspectionPlanUUID = o._targetInspectionPlanUUID;

        _autoHeadingMode = o._autoHeadingMode;

        return *this;
    }

    bool operator==(const GuiDTO& o) const Q_DECL_NOEXCEPT
    {
        return _missionName == o._missionName && _missionDescription == o._missionDescription
            && _missionCreationDate == o._missionCreationDate && _missionLastUpdateDate == o._missionLastUpdateDate
            && _hasRoutes == o._hasRoutes && _loadedHome == o._loadedHome && _appStatus == o._appStatus
            && _viewMode == o._viewMode && _missionStatus == o._missionStatus && _routeOptions == o._routeOptions
            && _loadedRoute == o._loadedRoute && _commsIsConnected == o._commsIsConnected &&

               _visualizationConfig == o._visualizationConfig &&

               _mavsdkCommsOptions == o._mavsdkCommsOptions && _ddhlCommsOptions == o._ddhlCommsOptions
            && _checkList == o._checkList && _inspectionPlan == o._inspectionPlan
            && _missionResults == o._missionResults && _downloadAssetIsCanceled == o._downloadAssetIsCanceled
            && _targetInspectionPlanUUID == o._targetInspectionPlanUUID &&

               _autoHeadingMode == o._autoHeadingMode;
    }

    bool operator!=(const GuiDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    QString   _missionName;
    QString   _missionDescription;
    QDateTime _missionCreationDate;
    QDateTime _missionLastUpdateDate;

    bool    _commsIsConnected{false};
    bool    _hasRoutes{false};
    HomeDTO _loadedHome;

    ViewModeType      _viewMode{ViewModeType::ThreeD};
    AppStatusType     _appStatus{AppStatusType::Panning};
    MissionStatusType _missionStatus{MissionStatusType::NoMission};

    RouteOptions _routeOptions{ShowLocal | ShowLoaded};
    RouteDTO     _loadedRoute;

    VisualizationConfigDTO _visualizationConfig;

    CommsLinkMavsdkDTO                _mavsdkCommsOptions;
    CommsLinkDDHLDTO                  _ddhlCommsOptions;
    CheckListDTO                      _checkList;
    InspectionPlanDTO                 _inspectionPlan;
    QSharedPointer<MissionResultsDTO> _missionResults;

    bool    _downloadAssetIsCanceled{false};
    QString _targetInspectionPlanUUID{""};

    bool _autoHeadingMode{false};
};

} // namespace gcs
