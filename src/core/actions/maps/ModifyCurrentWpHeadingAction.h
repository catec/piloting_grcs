#pragma once

#include "../BasicAction.h"

namespace gcs {
class IMap;

class PILOTING_GRCS_CORE_EXPORT ModifyCurrentWpHeadingAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit ModifyCurrentWpHeadingAction(IMap& map, QObject* parent = nullptr);
    virtual ~ModifyCurrentWpHeadingAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    IMap&    _map;
    QObject* _mapObj{nullptr};
};

} // namespace gcs
