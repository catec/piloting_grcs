#pragma once

#include <common/IDTO.h>
#include <common/Types.h>

#include <QtGlobal>

namespace gcs {

class BasicWaypointDTO : public IDTO
{
  public:
    virtual ~BasicWaypointDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::BASIC_WAYPOINT; }

    quint16& getId() Q_DECL_NOEXCEPT { return _id; }

    const quint16& getId() const Q_DECL_NOEXCEPT { return _id; }

    WaypointType& getWpType() Q_DECL_NOEXCEPT { return _wpType; }

    const WaypointType& getWpType() const Q_DECL_NOEXCEPT { return _wpType; }

    /// Operators
    BasicWaypointDTO& operator=(const BasicWaypointDTO& o) Q_DECL_NOEXCEPT
    {
        _id     = o._id;
        _wpType = o._wpType;

        return *this;
    }

    bool operator==(const BasicWaypointDTO& o) const Q_DECL_NOEXCEPT { return _id == o._id && _wpType == o._wpType; }

    bool operator!=(const BasicWaypointDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    quint16      _id{0};
    WaypointType _wpType{WaypointType::POSE};
};

} // namespace gcs
