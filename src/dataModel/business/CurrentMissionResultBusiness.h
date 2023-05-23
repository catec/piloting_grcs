#pragma once

#include <QString>

#include "gcs_dataModel_export.h"

namespace gcs {

class CurrentMissionResultDTO;
class RouteDTO;
class MissionTaskDTO;
class InspectionTaskDTO;
class CheckListDTO;

class PILOTING_GRCS_DATAMODEL_EXPORT CurrentMissionResultBusiness
{
  public:
    explicit CurrentMissionResultBusiness(CurrentMissionResultDTO&);
    virtual ~CurrentMissionResultBusiness();

    void newMission(const QString&, const QString&);
    void closeMission();

    void saveLoadedRouteToFile(const RouteDTO&);
    void saveCheckListToFile(const CheckListDTO&);
    void saveMissionResultToJson();

    void updateMissionTask(const MissionTaskDTO&);

    void createMissionTaskListFromRoute(const RouteDTO&, const QList<InspectionTaskDTO>&);
    void generateNewSyncId();

  private:
    void saveByteArrayDataInFile(const QString&, const QByteArray&);

    CurrentMissionResultDTO& _currentMissionResultDTO;
};

} // namespace gcs
