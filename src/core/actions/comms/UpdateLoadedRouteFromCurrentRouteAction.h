#pragma once

#include <dataModel/business/CurrentMissionResultBusiness.h>
#include <dataModel/business/RouteBusiness.h>

#include "../BasicAction.h"
#include "gcs_core_export.h"

namespace gcs {
class DataModel;

class PILOTING_GRCS_CORE_EXPORT UpdateLoadedRouteFromCurrentRouteAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit UpdateLoadedRouteFromCurrentRouteAction(DataModel& dataModel, QObject* parent = nullptr);
    virtual ~UpdateLoadedRouteFromCurrentRouteAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  Q_SIGNALS:
    void sendDTO(QSharedPointer<IDTO>) override;

  private:
    DataModel&                   _dataModel;
    RouteBusiness                _routeBusiness;
    CurrentMissionResultBusiness _currentMissionResultBusiness;
};

} // namespace gcs
