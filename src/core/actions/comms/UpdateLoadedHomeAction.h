#pragma once

#include <dataModel/business/HomeBusiness.h>
#include <dataModel/business/MissionStatusBusiness.h>
#include <dataModel/dtos/HomeDTO.h>

#include "../BasicAction.h"
#include "gcs_core_export.h"

namespace gcs {
class DataModel;
class HomeSelectionDialog;

class PILOTING_GRCS_CORE_EXPORT UpdateLoadedHomeAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit UpdateLoadedHomeAction(DataModel& dataModel, QObject* parent = nullptr);
    virtual ~UpdateLoadedHomeAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  Q_SIGNALS:
    void sendDTO(QSharedPointer<IDTO>) override;

  private:
    DataModel&            _dataModel;
    HomeBusiness          _homeBusiness;
    MissionStatusBusiness _missionStatusBusiness;
};

} // namespace gcs
