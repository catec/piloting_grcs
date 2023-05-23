
#pragma once

#include <dataModel/business/ApplicationStatusBusiness.h>
#include <dataModel/business/ViewModeBusiness.h>

#include "../BasicAction.h"

namespace gcs {

class DataModel;

class PILOTING_GRCS_CORE_EXPORT ChangeViewModeAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit ChangeViewModeAction(DataModel& dataModel, QObject* parent = nullptr);
    virtual ~ChangeViewModeAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    DataModel&                _dataModel;
    ViewModeBusiness          _viewModeBusiness;
    ApplicationStatusBusiness _appStatusBusiness;
};

} // namespace gcs
