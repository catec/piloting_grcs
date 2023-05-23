#pragma once

#include "../BasicAction.h"

namespace gcs {

class DataModel;

class PILOTING_GRCS_CORE_EXPORT RemoveMeasuringPointsAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit RemoveMeasuringPointsAction(DataModel& dataModel, QObject* parent = nullptr);
    virtual ~RemoveMeasuringPointsAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    DataModel& _dataModel;
};

} // namespace gcs
