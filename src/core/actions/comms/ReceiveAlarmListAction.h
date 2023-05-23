
#pragma once

#include "../BasicAction.h"

namespace gcs {

class PILOTING_GRCS_CORE_EXPORT ReceiveAlarmListAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit ReceiveAlarmListAction(QObject* parent = nullptr);
    virtual ~ReceiveAlarmListAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;
};

} // namespace gcs
