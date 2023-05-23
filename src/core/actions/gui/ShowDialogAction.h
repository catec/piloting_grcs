#pragma once

#include "../BasicAction.h"

namespace gcs {

class DataModel;

class PILOTING_GRCS_CORE_EXPORT ShowDialogAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit ShowDialogAction(QObject* parent = nullptr);
    virtual ~ShowDialogAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;
};

} // namespace gcs
