#pragma once

#include "../BasicAction.h"

namespace gcs {

class PILOTING_GRCS_CORE_EXPORT UpdateMsgConsoleAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit UpdateMsgConsoleAction(QObject* parent = nullptr);
    virtual ~UpdateMsgConsoleAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;
};

} // namespace gcs
