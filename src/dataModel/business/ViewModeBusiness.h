#pragma once

#include <QtGlobal>

#include "dataModel/dtos/ViewModeDTO.h"
#include "gcs_dataModel_export.h"

namespace gcs {
class PILOTING_GRCS_DATAMODEL_EXPORT ViewModeBusiness
{
  public:
    explicit ViewModeBusiness(ViewModeDTO& viewModeDTO);
    virtual ~ViewModeBusiness();

    void changeMode(const ViewModeType& viewType) Q_DECL_NOEXCEPT;

  private:
    ViewModeDTO& _viewModeDTO;
};

} // namespace gcs
