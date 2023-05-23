#pragma once

#include <QString>

#include "JsonWaypointListFile.h"
#include "RouteBusiness.h"
#include "gcs_dataModel_export.h"

namespace gcs {

class MissionDTO;

class PILOTING_GRCS_DATAMODEL_EXPORT MissionBusiness
{
  public:
    explicit MissionBusiness(MissionDTO& missionDTO);
    virtual ~MissionBusiness();

    void newMission(const QString&);
    void closeMission() Q_DECL_NOEXCEPT;

    /// \note. Methods to save and load Waypoint List associated to Inspection Plan
    void saveMission(const QString&, const QString& = QString());
    void openMission(const QString&, const QString&);

  private:
    MissionDTO&          _missionDTO;
    RouteBusiness        _routeBusiness;
    JsonWaypointListFile _jsonWptsListFile;
};

} // namespace gcs
