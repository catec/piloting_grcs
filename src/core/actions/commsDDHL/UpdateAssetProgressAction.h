

#pragma once

#include <dataModel/business/AssetBusiness.h>

#include "../BasicAction.h"

namespace gcs {

class DataModel;
class PILOTING_GRCS_CORE_EXPORT UpdateAssetProgressAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit UpdateAssetProgressAction(DataModel& dataModel, QObject* parent = nullptr);
    virtual ~UpdateAssetProgressAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    DataModel&    _dataModel;
    AssetBusiness _assetBusiness;
};

} // namespace gcs
