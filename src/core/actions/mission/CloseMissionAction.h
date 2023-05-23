#pragma once

#include <dataModel/business/ApplicationStatusBusiness.h>
#include <dataModel/business/CurrentMissionResultBusiness.h>
#include <dataModel/business/EditionModeBusiness.h>
#include <dataModel/business/MissionStatusBusiness.h>

#include "../BasicAction.h"

namespace gcs {

class DataModel;

class PILOTING_GRCS_CORE_EXPORT CloseMissionAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit CloseMissionAction(DataModel& dataModel, QObject* parent = nullptr);
    virtual ~CloseMissionAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    DataModel&                   _dataModel;
    MissionStatusBusiness        _missionStatusBusiness;
    CurrentMissionResultBusiness _currentMissionResultBusiness;
    ApplicationStatusBusiness    _appStatusBusiness;
};

} // namespace gcs
