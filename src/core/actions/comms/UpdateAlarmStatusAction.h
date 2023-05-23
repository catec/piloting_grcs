
#pragma once

#include "../BasicAction.h"

namespace gcs {

class PILOTING_GRCS_CORE_EXPORT UpdateAlarmStatusAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit UpdateAlarmStatusAction(QObject* parent = nullptr);
    virtual ~UpdateAlarmStatusAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;
};

} // namespace gcs
