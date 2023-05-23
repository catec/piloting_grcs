#include "HomeBusiness.h"

#include <QsLog/QsLog.h>

#include "../dtos/MissionDTO.h"

namespace gcs {

HomeBusiness::HomeBusiness(MissionDTO& missionDTO) : _missionDTO(missionDTO)
{
    QLOG_TRACE() << "HomeBusiness::HomeBusiness()";
}

HomeBusiness::~HomeBusiness()
{
    QLOG_TRACE() << "HomeBusiness::~HomeBusiness()";
}

void HomeBusiness::updateLoadedHome(const HomeDTO& homeToUpdate) Q_DECL_NOEXCEPT
{
    QLOG_TRACE() << "HomeBusiness::updateLoadedHome()";

    _missionDTO.getLoadedHome() = homeToUpdate;

    QLOG_DEBUG() << "HomeBusiness::updateLoadedHome() - "
                    "X:"
                 << _missionDTO.getLoadedHome().getPosition().getX()
                 << "Y:" << _missionDTO.getLoadedHome().getPosition().getY()
                 << "Z:" << _missionDTO.getLoadedHome().getPosition().getZ();
}

void HomeBusiness::removeLoadedHome() Q_DECL_NOEXCEPT
{
    QLOG_TRACE() << "HomeBusiness::removeLoadedHome()";

    _missionDTO.getLoadedHome() = HomeDTO();
}

} // namespace gcs
