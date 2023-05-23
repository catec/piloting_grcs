#pragma once

#include "../BasicAction.h"

namespace gcs {

class PILOTING_GRCS_CORE_EXPORT UpdateCursorPositionAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit UpdateCursorPositionAction(QObject* parent = nullptr);
    virtual ~UpdateCursorPositionAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;
};

} // namespace gcs
