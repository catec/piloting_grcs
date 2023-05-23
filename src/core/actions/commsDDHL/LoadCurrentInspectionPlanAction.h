#pragma once

#include "../BasicAction.h"

namespace gcs {

class DataModel;

class PILOTING_GRCS_CORE_EXPORT LoadCurrentInspectionPlanAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit LoadCurrentInspectionPlanAction(DataModel& dataModel, QObject* parent = nullptr);
    virtual ~LoadCurrentInspectionPlanAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    DataModel& _dataModel;
};

} // namespace gcs
