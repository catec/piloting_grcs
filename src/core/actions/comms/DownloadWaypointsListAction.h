
#pragma once

#include "../BasicAction.h"

namespace gcs {

class ICommunications;

class PILOTING_GRCS_CORE_EXPORT DownloadWaypointsListAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit DownloadWaypointsListAction(ICommunications& comms, QObject* parent = nullptr);
    virtual ~DownloadWaypointsListAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    ICommunications& _comms;
    QObject*         _commsObj{nullptr};
};

} // namespace gcs
