#pragma once

#include <common/IDTO.h>
#include <common/Types.h>

namespace gcs {

class MissionStatusDTO : public IDTO
{
  public:
    virtual ~MissionStatusDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::MISSION_STATUS; }

    MissionStatusType& getStatus() Q_DECL_NOEXCEPT { return _status; }

    const MissionStatusType& getStatus() const Q_DECL_NOEXCEPT { return _status; }

    /// Operators
    MissionStatusDTO& operator=(const MissionStatusDTO& o) Q_DECL_NOEXCEPT
    {
        _status = o._status;

        return *this;
    }

    bool operator==(const MissionStatusDTO& o) const Q_DECL_NOEXCEPT { return _status == o._status; }

    bool operator!=(const MissionStatusDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    MissionStatusType _status{MissionStatusType::NoMission};
};

} // namespace gcs
