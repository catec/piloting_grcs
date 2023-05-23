#pragma once

#include "../BasicAction.h"
#include "dataModel/business/AssetBusiness.h"
#include "dataModel/business/InspectionPlanBusiness.h"

namespace gcs {

class DataModel;
class DDHLComms;
class AssetDTO;
class InspectionPlanDTO;

class PILOTING_GRCS_CORE_EXPORT UpdateInspectionPlanAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit UpdateInspectionPlanAction(DDHLComms& comms, DataModel& dataModel, QObject* parent = nullptr);
    virtual ~UpdateInspectionPlanAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    DataModel&             _dataModel;
    InspectionPlanBusiness _inspectionPlanBusiness;
    AssetBusiness          _assetBusiness;
    DDHLComms&             _comms;
    QObject*               _commsObj{nullptr};
};

} // namespace gcs
