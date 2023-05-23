#pragma once

#include "../BasicAction.h"

namespace gcs {

class ICommunications;
class IMap;

class PILOTING_GRCS_CORE_EXPORT CommsDisconnectAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit CommsDisconnectAction(ICommunications& comms, IMap& map, QObject* parent = nullptr);
    virtual ~CommsDisconnectAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    ICommunications& _comms;
    QObject*         _commsObj{nullptr};
    IMap&            _map;
    QObject*         _mapObj{nullptr};
};

} // namespace gcs
