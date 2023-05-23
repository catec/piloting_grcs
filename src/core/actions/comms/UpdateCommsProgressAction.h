#pragma once

#include "../BasicAction.h"

namespace gcs {

class PILOTING_GRCS_CORE_EXPORT UpdateCommsProgressAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit UpdateCommsProgressAction(QObject* parent = nullptr);
    virtual ~UpdateCommsProgressAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;
};

} // namespace gcs
