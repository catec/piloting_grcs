#pragma once

#include <dataModel/business/ApplicationStatusBusiness.h>
#include <dataModel/business/MissionStatusBusiness.h>
#include <dataModel/business/RouteBusiness.h>

#include "../BasicAction.h"

namespace gcs {

class DataModel;

class PILOTING_GRCS_CORE_EXPORT CopyLoadedRouteToLocalAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit CopyLoadedRouteToLocalAction(DataModel& dataModel, QObject* parent = nullptr);
    virtual ~CopyLoadedRouteToLocalAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    DataModel&                _dataModel;
    RouteBusiness             _routeBusiness;
    MissionStatusBusiness     _missionStatusBusiness;
    ApplicationStatusBusiness _appStatusBusiness;
};

} // namespace gcs
