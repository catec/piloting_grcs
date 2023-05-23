#include "EditionModeBusiness.h"

#include <QsLog/QsLog.h>

namespace gcs {

EditionModeBusiness::EditionModeBusiness(EditionModeDTO& editionModeDTO) : _editionModeDTO(editionModeDTO)
{
    QLOG_TRACE() << "EditionModeBusiness::EditionModeBusiness()";
}

EditionModeBusiness::~EditionModeBusiness()
{
    QLOG_TRACE() << "EditionModeBusiness::~EditionModeBusiness()";
}

void EditionModeBusiness::changeMode(const bool& editionType) Q_DECL_NOEXCEPT
{
    QLOG_TRACE() << "EditionModeBusiness::changeMode()";

    QLOG_DEBUG() << "EditionModeBusiness::changeMode() - "
                    "mode: "
                 << (editionType ? "edition" : "view");

    _editionModeDTO.getMode() = editionType;
}

} // namespace gcs
