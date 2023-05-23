#include "DataModel.h"

#include <QsLog/QsLog.h>

namespace gcs {

DataModel::DataModel()
{
    QLOG_TRACE() << "DataModel::DataModel()";
}

DataModel::~DataModel()
{
    QLOG_TRACE() << "DataModel::~DataModel()";
}

MissionDTO& DataModel::getMissionDTO()
{
    return _missionDTO;
}

MissionStatusDTO& DataModel::getMissionStatusDTO()
{
    return _missionStatusDTO;
}

CurrentMissionResultDTO& DataModel::getCurrentMissionResultDTO()
{
    return _currentMissionResultDTO;
}

MissionResultsDTO& DataModel::getMissionResultsDTO()
{
    return _missionResultsDTO;
}

ApplicationStatusDTO& DataModel::getApplicationStatusDTO()
{
    return _appStatusDTO;
}

EditionModeDTO& DataModel::getEditionModeDTO()
{
    return _editionModeDTO;
}

ViewModeDTO& DataModel::getViewModeDTO()
{
    return _viewModeDTO;
}

RouteOptionsDTO& DataModel::getRouteOptionsDTO()
{
    return _routeOptionsDTO;
}

MeasureDTO& DataModel::getMeasureDTO()
{
    return _measureDTO;
}

CommsStatusDTO& DataModel::getCommsStatusDTO()
{
    return _commsStatusDTO;
}

BatteryParametersDTO& DataModel::getBatteryParametersDTO()
{
    return _batteryParametersDTO;
}

InspectionPlanDTO& DataModel::getInspectionPlanDTO()
{
    return _inspectionPlanDTO;
}

CurrentInspItemDTO& DataModel::getCurrentInspItemDTO()
{
    return _currentInspItemDTO;
}

ReachedInspItemDTO& DataModel::getReachedInspItemDTO()
{
    return _reachedInspItemDTO;
}

SupportedCommandListDTO& DataModel::getSupportedCommandListDTO()
{
    return _supportedCommandListDTO;
}

RobotPositionVelocityNedDTO& DataModel::getRobotPositionVelocityDTO()
{
    return _robotPositionVelNedDTO;
}

RobotAttitudeAngVelocityDTO& DataModel::getRobotAttitudeAngVelocityDTO()
{
    return _robotAttitudeAngVelocityDTO;
}

CommsLinkDDHLDTO& DataModel::getCommsLinkDDHLDTO()
{
    return _commsLinkDDHLDTO;
}

bool& DataModel::getAutoHeadingMode()
{
    return _autoHeadingMode;
}

} // namespace gcs
