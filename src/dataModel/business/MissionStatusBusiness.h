#pragma once

#include <common/Types.h>

#include <QtGlobal>

#include "gcs_dataModel_export.h"

namespace gcs {

class MissionStatusDTO;

class PILOTING_GRCS_DATAMODEL_EXPORT MissionStatusBusiness
{
  public:
    explicit MissionStatusBusiness(MissionStatusDTO& missionStatusDTO);
    virtual ~MissionStatusBusiness();

    void changeStatus(const MissionStatusType&) Q_DECL_NOEXCEPT;

  private:
    MissionStatusDTO& _missionStatusDTO;
};

} // namespace gcs
