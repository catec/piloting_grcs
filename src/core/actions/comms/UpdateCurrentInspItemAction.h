
#pragma once

#include "../BasicAction.h"

namespace gcs {
class DataModel;

class PILOTING_GRCS_CORE_EXPORT UpdateCurrentInspItemAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit UpdateCurrentInspItemAction(DataModel& dataModel, QObject* parent = nullptr);
    virtual ~UpdateCurrentInspItemAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    DataModel& _dataModel;
};

} // namespace gcs
