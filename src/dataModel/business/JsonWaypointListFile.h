#pragma once

#include "JsonCheckKeys.h"
#include "gcs_dataModel_export.h"

class QString;

namespace gcs {

class MissionDTO;

class PILOTING_GRCS_DATAMODEL_EXPORT JsonWaypointListFile
{
  public:
    explicit JsonWaypointListFile(MissionDTO& missionDTO);
    virtual ~JsonWaypointListFile();

    void load(const QString&, const QString&);
    void save(const QString&, const QString&);

  private:
    MissionDTO&   _missionDTO;
    JsonCheckKeys _jsonCheckKeys;
};

} // namespace gcs
