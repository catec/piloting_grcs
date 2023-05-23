#pragma once

#include <dataModel/business/ApplicationStatusBusiness.h>
#include <dataModel/business/CurrentMissionResultBusiness.h>
#include <dataModel/business/MissionBusiness.h>
#include <dataModel/business/MissionStatusBusiness.h>

#include "../BasicAction.h"

namespace gcs {

class DataModel;

class PILOTING_GRCS_CORE_EXPORT NewMissionAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit NewMissionAction(DataModel& dataModel, QObject* parent = nullptr);
    virtual ~NewMissionAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    DataModel&                   _dataModel;
    MissionBusiness              _missionBusiness;
    MissionStatusBusiness        _missionStatusBusiness;
    CurrentMissionResultBusiness _currentMissionResultBusiness;
    ApplicationStatusBusiness    _appStatusBusiness;
};

} // namespace gcs
