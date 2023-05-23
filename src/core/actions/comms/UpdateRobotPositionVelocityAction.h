
#pragma once

#include "../BasicAction.h"

namespace gcs {
class DataModel;

class PILOTING_GRCS_CORE_EXPORT UpdateRobotPositionVelocityAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit UpdateRobotPositionVelocityAction(DataModel& dataModel, QObject* parent = nullptr);
    virtual ~UpdateRobotPositionVelocityAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    DataModel& _dataModel;
};

} // namespace gcs
