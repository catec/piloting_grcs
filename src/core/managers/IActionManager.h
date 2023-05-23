#pragma once

#include <core/IAction.h>

#include <memory>

#include "gcs_core_export.h"

namespace gcs {

class PILOTING_GRCS_CORE_EXPORT IActionManager
{
  public:
    virtual ~IActionManager() {}

    virtual int getSize() = 0;

    virtual void addAction(std::unique_ptr<IAction>) = 0;

    virtual void removeAction(const ActionType&) = 0;

    virtual void removeAllActions() = 0;

    virtual void executeAction(const ActionType&, QSharedPointer<IDTO>) = 0;

    /// Q_SIGNALS:

    virtual void sendDTO(QSharedPointer<IDTO>) = 0;
};

} // namespace gcs
