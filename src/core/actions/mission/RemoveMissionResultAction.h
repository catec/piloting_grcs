#pragma once

#include <dataModel/business/MissionResultsBusiness.h>

#include "../BasicAction.h"

namespace gcs {

class DataModel;

class PILOTING_GRCS_CORE_EXPORT RemoveMissionResultAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit RemoveMissionResultAction(DataModel& dataModel, QObject* parent = nullptr);
    virtual ~RemoveMissionResultAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    DataModel&             _dataModel;
    MissionResultsBusiness _missionResultsBusiness;
};

} // namespace gcs
