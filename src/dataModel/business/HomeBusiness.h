#pragma once

#include <QtGlobal>

#include "gcs_dataModel_export.h"

namespace gcs {

class MissionDTO;
class HomeDTO;

class PILOTING_GRCS_DATAMODEL_EXPORT HomeBusiness
{
  public:
    explicit HomeBusiness(MissionDTO& missionDTO);
    virtual ~HomeBusiness();

    void updateLoadedHome(const HomeDTO&) Q_DECL_NOEXCEPT;
    void removeLoadedHome() Q_DECL_NOEXCEPT;

  private:
    MissionDTO& _missionDTO;
};

} // namespace gcs
