#pragma once

#include <dataModel/business/ApplicationStatusBusiness.h>
#include <dataModel/business/MissionBusiness.h>
#include <dataModel/business/MissionStatusBusiness.h>

#include "../BasicAction.h"
#include "gcs_core_export.h"

namespace gcs {
class DataModel;

class PILOTING_GRCS_CORE_EXPORT LoadAssociatedWaypointListAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit LoadAssociatedWaypointListAction(DataModel& dataModel, QObject* parent = nullptr);
    virtual ~LoadAssociatedWaypointListAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  Q_SIGNALS:
    void sendDTO(QSharedPointer<IDTO>) override;

  private:
    DataModel&            _dataModel;
    MissionBusiness       _missionBusiness;
    MissionStatusBusiness _missionStatusBusiness;
};

} // namespace gcs
