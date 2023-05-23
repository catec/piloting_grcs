#pragma once

#include "../BasicAction.h"

namespace gcs {
class IMap;

class PILOTING_GRCS_CORE_EXPORT HidePointCloudAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit HidePointCloudAction(IMap& map, QObject* parent = nullptr);
    virtual ~HidePointCloudAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    IMap&    _map;
    QObject* _mapObj{nullptr};
};

} // namespace gcs
