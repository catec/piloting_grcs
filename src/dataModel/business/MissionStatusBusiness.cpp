#include "MissionStatusBusiness.h"

#include <QsLog/QsLog.h>

#include "../dtos/MissionStatusDTO.h"

namespace gcs {

MissionStatusBusiness::MissionStatusBusiness(MissionStatusDTO& missionStatusDTO) : _missionStatusDTO(missionStatusDTO)
{
    QLOG_TRACE() << "MissionStatusBusiness::MissionStatusBusiness()";
}

MissionStatusBusiness::~MissionStatusBusiness()
{
    QLOG_TRACE() << "MissionStatusBusiness::~MissionStatusBusiness()";
}

void MissionStatusBusiness::changeStatus(const MissionStatusType& status) Q_DECL_NOEXCEPT
{
    QLOG_TRACE() << "MissionStatusBusiness::changeStatus()";

    QLOG_DEBUG() << "MissionStatusBusiness::changeStatus() - "
                    "Status:"
                 << ToString(status);

    _missionStatusDTO.getStatus() = status;
}

} // namespace gcs
