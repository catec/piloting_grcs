#pragma once

#include "../BasicAction.h"
#include "dataModel/business/AssetBusiness.h"
#include "dataModel/business/InspectionPlanBusiness.h"

namespace gcs {

class DataModel;
class AssetDTO;

class PILOTING_GRCS_CORE_EXPORT UpdateAssetPathAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit UpdateAssetPathAction(DataModel& dataModel, QObject* parent = nullptr);
    virtual ~UpdateAssetPathAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    DataModel&             _dataModel;
    InspectionPlanBusiness _inspectionPlanBusiness;
    AssetBusiness          _assetBusiness;
};

} // namespace gcs
