#pragma once

#include <dataModel/business/RouteBusiness.h>

#include "../BasicAction.h"
#include "gcs_core_export.h"

namespace gcs {
class DataModel;

class PILOTING_GRCS_CORE_EXPORT RemoveLoadedRouteAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit RemoveLoadedRouteAction(DataModel& dataModel, QObject* parent = nullptr);
    virtual ~RemoveLoadedRouteAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  Q_SIGNALS:
    void sendDTO(QSharedPointer<IDTO>) override;

  private:
    DataModel&    _dataModel;
    RouteBusiness _routeBusiness;
};

} // namespace gcs
