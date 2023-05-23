#pragma once

#include <QString>

#include "JsonCheckKeys.h"
#include "gcs_dataModel_export.h"

namespace gcs {

class MissionResultsDTO;
class MissionResultDTO;

class PILOTING_GRCS_DATAMODEL_EXPORT MissionResultsBusiness
{
  public:
    explicit MissionResultsBusiness(MissionResultsDTO&);
    virtual ~MissionResultsBusiness();

    void loadMissionResults(const QString&);

    void removeMissionResult(const MissionResultDTO&);

    void obtainMissionResultList(const QStringList&, QList<MissionResultDTO>&) const;

  private:
    MissionResultDTO convertMissionFileToDTO(const QString&);
    void             copyAndRemoveMissionFolder(const QString&, const QString&);
    void             createMissionBackUpFolder(const QString&);

    MissionResultsDTO& _missionResultsDTO;
    JsonCheckKeys      _jsonCheckKeys;
};

} // namespace gcs
