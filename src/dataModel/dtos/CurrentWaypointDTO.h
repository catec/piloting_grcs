#pragma once

#include <common/IDTO.h>

#include <QtGlobal>

namespace gcs {

class CurrentWaypointDTO : public IDTO
{
  public:
    virtual ~CurrentWaypointDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::CURRENT_WAYPOINT; }

    quint16& getSeqNumber() Q_DECL_NOEXCEPT { return _seqNumber; }

    const quint16& getSeqNumber() const Q_DECL_NOEXCEPT { return _seqNumber; }

    /// Operators
    CurrentWaypointDTO& operator=(const CurrentWaypointDTO& o) Q_DECL_NOEXCEPT
    {
        _seqNumber = o._seqNumber;

        return *this;
    }

    bool operator==(const CurrentWaypointDTO& o) const Q_DECL_NOEXCEPT { return _seqNumber == o._seqNumber; }

    bool operator!=(const CurrentWaypointDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    quint16 _seqNumber{0};
};

} // namespace gcs
