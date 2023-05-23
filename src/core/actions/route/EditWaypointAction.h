#pragma once

#include <dataModel/business/MissionStatusBusiness.h>
#include <dataModel/business/RouteBusiness.h>

#include "../BasicAction.h"

namespace gcs {

class DataModel;

class PILOTING_GRCS_CORE_EXPORT EditWaypointAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit EditWaypointAction(DataModel& dataModel, QObject* parent = nullptr);
    virtual ~EditWaypointAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    DataModel&            _dataModel;
    RouteBusiness         _routeBusiness;
    MissionStatusBusiness _missionStatusBusiness;
};

} // namespace gcs
