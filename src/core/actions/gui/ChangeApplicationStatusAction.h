#pragma once

#include <dataModel/business/ApplicationStatusBusiness.h>

#include "../BasicAction.h"

namespace gcs {

class DataModel;

class PILOTING_GRCS_CORE_EXPORT ChangeApplicationStatusAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit ChangeApplicationStatusAction(DataModel& dataModel, QObject* parent = nullptr);
    virtual ~ChangeApplicationStatusAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    DataModel&                _dataModel;
    ApplicationStatusBusiness _appStatusBusiness;
};

} // namespace gcs
