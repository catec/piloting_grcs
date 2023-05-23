#pragma once

#include "../BasicAction.h"

namespace gcs {

class DataModel;

class PILOTING_GRCS_CORE_EXPORT UpdateCloudDimensionsAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit UpdateCloudDimensionsAction(QObject* parent = nullptr);
    virtual ~UpdateCloudDimensionsAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;
};

} // namespace gcs
