#pragma once

#include <communications/MavSDK/dtos/CurrentInspItemDTO.h>
#include <communications/MavSDK/dtos/ReachedInspItemDTO.h>
#include <communications/MavSDK/dtos/SupportedCommandListDTO.h>
#include <dataModel/RobotAttitudeAngVelocityDTO.h>
#include <dataModel/RobotPositionVelocityNedDTO.h>

#include "dtos/ApplicationStatusDTO.h"
#include "dtos/AssetDTO.h"
#include "dtos/BatteryParametersDTO.h"
#include "dtos/CommsLinkDDHLDTO.h"
#include "dtos/CommsStatusDTO.h"
#include "dtos/CurrentMissionResultDTO.h"
#include "dtos/EditionModeDTO.h"
#include "dtos/InspectionPlanDTO.h"
#include "dtos/MeasureDTO.h"
#include "dtos/MissionDTO.h"
#include "dtos/MissionResultsDTO.h"
#include "dtos/MissionStatusDTO.h"
#include "dtos/RouteOptionsDTO.h"
#include "dtos/ViewModeDTO.h"
#include "gcs_dataModel_export.h"

namespace gcs {

class PILOTING_GRCS_DATAMODEL_EXPORT DataModel
{
  public:
    explicit DataModel();
    virtual ~DataModel();

    /// Accessors
    MissionDTO&              getMissionDTO();
    MissionStatusDTO&        getMissionStatusDTO();
    CurrentMissionResultDTO& getCurrentMissionResultDTO();
    MissionResultsDTO&       getMissionResultsDTO();
    ApplicationStatusDTO&    getApplicationStatusDTO();
    EditionModeDTO&          getEditionModeDTO();
    ViewModeDTO&             getViewModeDTO();
    RouteOptionsDTO&         getRouteOptionsDTO();
    MeasureDTO&              getMeasureDTO();
    CommsStatusDTO&          getCommsStatusDTO();
    BatteryParametersDTO&    getBatteryParametersDTO();

    InspectionPlanDTO& getInspectionPlanDTO();

    CurrentInspItemDTO&      getCurrentInspItemDTO();
    ReachedInspItemDTO&      getReachedInspItemDTO();
    SupportedCommandListDTO& getSupportedCommandListDTO();

    RobotPositionVelocityNedDTO& getRobotPositionVelocityDTO();
    RobotAttitudeAngVelocityDTO& getRobotAttitudeAngVelocityDTO();
    CommsLinkDDHLDTO&            getCommsLinkDDHLDTO();

    bool& getAutoHeadingMode();

  private:
    MissionDTO              _missionDTO;
    MissionStatusDTO        _missionStatusDTO;
    CurrentMissionResultDTO _currentMissionResultDTO;
    MissionResultsDTO       _missionResultsDTO;
    ApplicationStatusDTO    _appStatusDTO;
    EditionModeDTO          _editionModeDTO;
    ViewModeDTO             _viewModeDTO;
    RouteOptionsDTO         _routeOptionsDTO;
    MeasureDTO              _measureDTO;
    CommsStatusDTO          _commsStatusDTO;
    BatteryParametersDTO    _batteryParametersDTO;

    InspectionPlanDTO _inspectionPlanDTO;

    CurrentInspItemDTO      _currentInspItemDTO;
    ReachedInspItemDTO      _reachedInspItemDTO;
    SupportedCommandListDTO _supportedCommandListDTO;

    RobotPositionVelocityNedDTO _robotPositionVelNedDTO;
    RobotAttitudeAngVelocityDTO _robotAttitudeAngVelocityDTO;
    CommsLinkDDHLDTO            _commsLinkDDHLDTO;

    bool _autoHeadingMode;
};

} // namespace gcs
