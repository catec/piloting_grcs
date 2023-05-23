
#pragma once

#include "../BasicAction.h"

namespace gcs {

class DDHLComms;

class PILOTING_GRCS_CORE_EXPORT CancelAssetDownloadAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit CancelAssetDownloadAction(DDHLComms& comms, QObject* parent = nullptr);
    virtual ~CancelAssetDownloadAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    DDHLComms& _comms;
    QObject*   _commsObj{nullptr};
};

} // namespace gcs
