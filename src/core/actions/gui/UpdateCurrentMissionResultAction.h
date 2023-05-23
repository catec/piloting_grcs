
#pragma once

#include <dataModel/business/CurrentMissionResultBusiness.h>

#include "../BasicAction.h"

namespace gcs {

class DataModel;

class PILOTING_GRCS_CORE_EXPORT UpdateCurrentMissionResultAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit UpdateCurrentMissionResultAction(DataModel& dataModel, QObject* parent = nullptr);
    virtual ~UpdateCurrentMissionResultAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    DataModel&                   _dataModel;
    CurrentMissionResultBusiness _currentMissionResultBusiness;
};

} // namespace gcs