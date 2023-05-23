
#pragma once

#include "../BasicAction.h"

namespace gcs {
class DataModel;

class PILOTING_GRCS_CORE_EXPORT UpdateRobotAttitudeAngVelocityAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit UpdateRobotAttitudeAngVelocityAction(DataModel& dataModel, QObject* parent = nullptr);
    virtual ~UpdateRobotAttitudeAngVelocityAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    DataModel& _dataModel;
};

} // namespace gcs
