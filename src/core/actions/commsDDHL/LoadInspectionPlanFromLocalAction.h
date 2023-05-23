#pragma once

#include "../BasicAction.h"
#include "dataModel/business/InspectionPlanBusiness.h"

namespace gcs {

class DataModel;
class DDHLComms;
class InspectionPlanDTO;

class PILOTING_GRCS_CORE_EXPORT LoadInspectionPlanFromLocalAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit LoadInspectionPlanFromLocalAction(DDHLComms& comms, DataModel& dataModel, QObject* parent = nullptr);
    virtual ~LoadInspectionPlanFromLocalAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    DataModel&             _dataModel;
    InspectionPlanBusiness _inspectionPlanBusiness;
    DDHLComms&             _comms;
    QObject*               _commsObj{nullptr};
};

} // namespace gcs
