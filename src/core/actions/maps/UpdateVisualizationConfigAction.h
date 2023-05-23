#pragma once

#include "../BasicAction.h"

namespace gcs {

class PILOTING_GRCS_CORE_EXPORT UpdateVisualizationConfigAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit UpdateVisualizationConfigAction(QObject* parent = nullptr);
    virtual ~UpdateVisualizationConfigAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;
};

} // namespace gcs
