
#pragma once

#include "../BasicAction.h"

namespace gcs {

class DDHLComms;
class DataModel;

class PILOTING_GRCS_CORE_EXPORT DownloadAssetFromDDHLAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit DownloadAssetFromDDHLAction(DDHLComms& comms, DataModel& dataModel, QObject* parent = nullptr);
    virtual ~DownloadAssetFromDDHLAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    DataModel& _dataModel;
    DDHLComms& _comms;
    QObject*   _commsObj{nullptr};
};

} // namespace gcs
