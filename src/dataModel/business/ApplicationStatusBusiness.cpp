#include "ApplicationStatusBusiness.h"

#include <QsLog/QsLog.h>

#include "../dtos/ApplicationStatusDTO.h"

namespace gcs {

ApplicationStatusBusiness::ApplicationStatusBusiness(ApplicationStatusDTO& appStatusDTO) : _appStatusDTO(appStatusDTO)
{
    QLOG_TRACE() << "ApplicationStatusBusiness::ApplicationStatusBusiness()";
}

ApplicationStatusBusiness::~ApplicationStatusBusiness()
{
    QLOG_TRACE() << "ApplicationStatusBusiness::~ApplicationStatusBusiness()";
}

void ApplicationStatusBusiness::changeStatus(const AppStatusType& statusType) Q_DECL_NOEXCEPT
{
    QLOG_TRACE() << "ApplicationStatusBusiness::changeStatus()";

    QLOG_DEBUG() << "ApplicationStatusBusiness::changeStatus() - "
                    "Status:"
                 << ToString(statusType);

    _appStatusDTO.getStatus() = statusType;
}

} // namespace gcs
