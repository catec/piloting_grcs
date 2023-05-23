#pragma once

#include <QtGlobal>

#include "dataModel/dtos/EditionModeDTO.h"
#include "gcs_dataModel_export.h"

namespace gcs {
class PILOTING_GRCS_DATAMODEL_EXPORT EditionModeBusiness
{
  public:
    explicit EditionModeBusiness(EditionModeDTO& editionModeDTO);
    virtual ~EditionModeBusiness();

    void changeMode(const bool& editionType) Q_DECL_NOEXCEPT;

  private:
    EditionModeDTO& _editionModeDTO;
};

} // namespace gcs
