
#pragma once

#include "../BasicAction.h"

namespace gcs {
class DataModel;

class PILOTING_GRCS_CORE_EXPORT UpdateReachedInspItemAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit UpdateReachedInspItemAction(DataModel& dataModel, QObject* parent = nullptr);
    virtual ~UpdateReachedInspItemAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    DataModel& _dataModel;
};

} // namespace gcs
