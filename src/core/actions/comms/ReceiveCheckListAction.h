
#pragma once

#include <dataModel/business/CurrentMissionResultBusiness.h>

#include "core/actions/BasicAction.h"
namespace gcs {
class DataModel;

class PILOTING_GRCS_CORE_EXPORT ReceiveCheckListAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit ReceiveCheckListAction(DataModel& dataModel, QObject* parent = nullptr);
    virtual ~ReceiveCheckListAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    DataModel&                   _dataModel;
    CurrentMissionResultBusiness _currentMissionResultBusiness;
};

} // namespace gcs
