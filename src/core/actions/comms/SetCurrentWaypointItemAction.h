
#pragma once

#include "../BasicAction.h"

namespace gcs {

class ICommunications;

class PILOTING_GRCS_CORE_EXPORT SetCurrentWaypointItemAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit SetCurrentWaypointItemAction(ICommunications& comms, QObject* parent = nullptr);
    virtual ~SetCurrentWaypointItemAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    ICommunications& _comms;
    QObject*         _commsObj{nullptr};
};

} // namespace gcs
