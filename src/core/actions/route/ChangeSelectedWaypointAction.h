#pragma once

#include <dataModel/business/RouteBusiness.h>

#include "../BasicAction.h"

namespace gcs {

class DataModel;

class PILOTING_GRCS_CORE_EXPORT ChangeSelectedWaypointAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit ChangeSelectedWaypointAction(DataModel& dataModel, QObject* parent = nullptr);
    virtual ~ChangeSelectedWaypointAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    DataModel&    _dataModel;
    RouteBusiness _routeBusiness;
};

} // namespace gcs
