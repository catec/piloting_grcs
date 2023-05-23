#pragma once

#include "../BasicAction.h"

namespace gcs {

class DataModel;

class PILOTING_GRCS_CORE_EXPORT AddMeasuringPointAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit AddMeasuringPointAction(DataModel& dataModel, QObject* parent = nullptr);
    virtual ~AddMeasuringPointAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    DataModel& _dataModel;
};

} // namespace gcs
