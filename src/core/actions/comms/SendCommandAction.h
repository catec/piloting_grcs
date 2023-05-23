
#pragma once

#include "../BasicAction.h"

namespace gcs {

class ICommunications;

class PILOTING_GRCS_CORE_EXPORT SendCommandAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit SendCommandAction(ICommunications& comms, QObject* parent = nullptr);
    virtual ~SendCommandAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    ICommunications& _comms;
    QObject*         _commsObj{nullptr};
};

} // namespace gcs
