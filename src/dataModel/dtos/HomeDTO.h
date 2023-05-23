#pragma once

#include "common/IDTO.h"
#include "dataModel/Position3dDTO.h"

namespace gcs {

class HomeDTO : public IDTO
{
  public:
    virtual ~HomeDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::HOME; }

    bool& getIsEnabled() Q_DECL_NOEXCEPT { return _isEnabled; }

    const bool& getIsEnabled() const Q_DECL_NOEXCEPT { return _isEnabled; }

    Position3dDTO& getPosition() Q_DECL_NOEXCEPT { return _position3D; }

    const Position3dDTO& getPosition() const Q_DECL_NOEXCEPT { return _position3D; }

    float& getYawOrientation() Q_DECL_NOEXCEPT { return _yaw_orientation; }

    const float& getYawOrientation() const Q_DECL_NOEXCEPT { return _yaw_orientation; }

    /// Operators
    HomeDTO& operator=(const HomeDTO& o) Q_DECL_NOEXCEPT
    {
        _isEnabled       = o._isEnabled;
        _position3D      = o._position3D;
        _yaw_orientation = o._yaw_orientation;

        return *this;
    }

    bool operator==(const HomeDTO& o) const Q_DECL_NOEXCEPT
    {
        return _isEnabled == o._isEnabled && _position3D == o._position3D && _yaw_orientation == o._yaw_orientation;
    }

    bool operator!=(const HomeDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    bool          _isEnabled{false};
    Position3dDTO _position3D;      /// meters
    float         _yaw_orientation; /// degrees
};

} // namespace gcs
