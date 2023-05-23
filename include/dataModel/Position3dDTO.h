#pragma once

#include <dataModel/Position2dDTO.h>

namespace gcs {

class Position3dDTO : public Position2dDTO
{
  public:
    virtual ~Position3dDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::POSITION_3D; }

    float& getZ() Q_DECL_NOEXCEPT { return _position_z; }

    const float& getZ() const Q_DECL_NOEXCEPT { return _position_z; }

    /// Operators
    Position3dDTO& operator=(const Position3dDTO& o) Q_DECL_NOEXCEPT
    {
        Position2dDTO::operator=(o);
        _position_z = o._position_z;

        return *this;
    }

    bool operator==(const Position3dDTO& o) const Q_DECL_NOEXCEPT
    {
        return Position2dDTO::operator==(o) && _position_z == o._position_z;
    }

    bool operator!=(const Position3dDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    float _position_z{0.0}; /// In meters
};

} // namespace gcs
