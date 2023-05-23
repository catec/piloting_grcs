
#pragma once

#include <dataModel/business/XmlDataBusiness.h>

#include "../BasicAction.h"

namespace gcs {
class DataModel;

class PILOTING_GRCS_CORE_EXPORT ReceiveHLActionsAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit ReceiveHLActionsAction(DataModel& dataModel, QObject* parent = nullptr);
    virtual ~ReceiveHLActionsAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    DataModel&      _dataModel;
    XmlDataBusiness _xmlDataBusiness;
};

} // namespace gcs
