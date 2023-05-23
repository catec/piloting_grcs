#pragma once

#include <dataModel/business/ApplicationStatusBusiness.h>
#include <dataModel/business/EditionModeBusiness.h>

#include "../BasicAction.h"

namespace gcs {

class DataModel;

class PILOTING_GRCS_CORE_EXPORT ChangeEditionModeAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit ChangeEditionModeAction(DataModel& dataModel, QObject* parent = nullptr);
    virtual ~ChangeEditionModeAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    DataModel&                _dataModel;
    EditionModeBusiness       _editionModeBusiness;
    ApplicationStatusBusiness _appStatusBusiness;
};

} // namespace gcs
