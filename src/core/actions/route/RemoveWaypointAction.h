#pragma once

#include <dataModel/business/ApplicationStatusBusiness.h>
#include <dataModel/business/MissionStatusBusiness.h>
#include <dataModel/business/RouteBusiness.h>

#include "../BasicAction.h"

namespace gcs {

class DataModel;

class PILOTING_GRCS_CORE_EXPORT RemoveWaypointAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit RemoveWaypointAction(DataModel& dataModel, QObject* parent = nullptr);
    virtual ~RemoveWaypointAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    DataModel&                _dataModel;
    RouteBusiness             _routeBusiness;
    MissionStatusBusiness     _missionStatusBusiness;
    ApplicationStatusBusiness _appStatusBusiness;
};

} // namespace gcs
