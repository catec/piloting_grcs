#pragma once

#include <dataModel/business/RouteOptionsBusiness.h>

#include "../BasicAction.h"

namespace gcs {

class DataModel;

class PILOTING_GRCS_CORE_EXPORT ChangeRouteOptionsAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit ChangeRouteOptionsAction(DataModel& dataModel, QObject* parent = nullptr);
    virtual ~ChangeRouteOptionsAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    DataModel&           _dataModel;
    RouteOptionsBusiness _routeOptionsBusiness;
};

} // namespace gcs
