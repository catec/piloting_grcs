
#include "ViewModeBusiness.h"

#include <QsLog/QsLog.h>

namespace gcs {

ViewModeBusiness::ViewModeBusiness(ViewModeDTO& viewModeDTO) : _viewModeDTO(viewModeDTO)
{
    QLOG_TRACE() << "ViewModeBusiness::ViewModeBusiness()";
}

ViewModeBusiness::~ViewModeBusiness()
{
    QLOG_TRACE() << "ViewModeBusiness::~ViewModeBusiness()";
}

void ViewModeBusiness::changeMode(const ViewModeType& viewType) Q_DECL_NOEXCEPT
{
    QLOG_TRACE() << "ViewModeBusiness::changeMode()";

    QLOG_DEBUG() << "ViewModeBusiness::changeMode() - "
                    "mode:"
                 << ToString(viewType);

    _viewModeDTO.getMode() = viewType;
}

} // namespace gcs
