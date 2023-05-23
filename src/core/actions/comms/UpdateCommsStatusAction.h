#pragma once

#include "../BasicAction.h"

namespace gcs {

class DataModel;

class PILOTING_GRCS_CORE_EXPORT UpdateCommsStatusAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit UpdateCommsStatusAction(DataModel& dataModel, QObject* parent = nullptr);
    virtual ~UpdateCommsStatusAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    DataModel& _dataModel;
};

} // namespace gcs
