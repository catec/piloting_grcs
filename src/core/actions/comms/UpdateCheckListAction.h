#pragma once

#include <dataModel/business/CurrentMissionResultBusiness.h>

#include "../BasicAction.h"
namespace gcs {
class DataModel;

class PILOTING_GRCS_CORE_EXPORT UpdateCheckListAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit UpdateCheckListAction(DataModel& dataModel, QObject* parent = nullptr);
    virtual ~UpdateCheckListAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    DataModel&                   _dataModel;
    CurrentMissionResultBusiness _currentMissionResultBusiness;
};

} // namespace gcs
