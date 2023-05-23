#pragma once

#include <dataModel/business/HomeBusiness.h>

#include "../BasicAction.h"
#include "gcs_core_export.h"

namespace gcs {
class DataModel;

class PILOTING_GRCS_CORE_EXPORT RemoveLoadedHomeAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit RemoveLoadedHomeAction(DataModel& dataModel, QObject* parent = nullptr);
    virtual ~RemoveLoadedHomeAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  Q_SIGNALS:
    void sendDTO(QSharedPointer<IDTO>) override;

  private:
    DataModel&   _dataModel;
    HomeBusiness _homeBusiness;
};

} // namespace gcs
