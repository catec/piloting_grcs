#pragma once

#include "../BasicAction.h"

namespace gcs {
class IMap;

class PILOTING_GRCS_CORE_EXPORT ModifyPreviewWpHeadingAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit ModifyPreviewWpHeadingAction(IMap& map, QObject* parent = nullptr);
    virtual ~ModifyPreviewWpHeadingAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    IMap&    _map;
    QObject* _mapObj{nullptr};
};

} // namespace gcs
