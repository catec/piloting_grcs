#pragma once

#include <common/Types.h>

#include <QtGlobal>

#include "gcs_dataModel_export.h"

namespace gcs {

class ApplicationStatusDTO;

class PILOTING_GRCS_DATAMODEL_EXPORT ApplicationStatusBusiness
{
  public:
    explicit ApplicationStatusBusiness(ApplicationStatusDTO& appStatusDTO);
    virtual ~ApplicationStatusBusiness();

    void changeStatus(const AppStatusType& stateType) Q_DECL_NOEXCEPT;

  private:
    ApplicationStatusDTO& _appStatusDTO;
};

} // namespace gcs
