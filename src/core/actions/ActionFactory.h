#pragma once

#include <core/IAction.h>

#include <memory>

#include "gcs_core_export.h"

namespace gcs {

class DataModel;
class ICommunications;
class DDHLComms;
class IMap;

class PILOTING_GRCS_CORE_EXPORT ActionFactory
{
  public:
    static std::unique_ptr<IAction> create(
            const ActionType& type,
            DataModel&        dataModel,
            ICommunications*  comms     = nullptr,
            DDHLComms*        commsDDHL = nullptr,
            IMap*             map       = nullptr);
};

} // namespace gcs
