
#pragma once

#include "../BasicAction.h"

namespace gcs {

class ICommunications;

class PILOTING_GRCS_CORE_EXPORT DownloadCheckListAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit DownloadCheckListAction(ICommunications& comms, QObject* parent = nullptr);
    virtual ~DownloadCheckListAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    ICommunications& _comms;
    QObject*         _commsObj{nullptr};
};

} // namespace gcs
