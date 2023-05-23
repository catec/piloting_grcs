#pragma once

#include <common/IDTO.h>

namespace gcs {

class Position2dDTO : public IDTO
{
  public:
    virtual ~Position2dDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::POSITION_2D; }

    float& getX() Q_DECL_NOEXCEPT { return _position_x; }

    const float& getX() const Q_DECL_NOEXCEPT { return _position_x; }

    float& getY() Q_DECL_NOEXCEPT { return _position_y; }

    const float& getY() const Q_DECL_NOEXCEPT { return _position_y; }

    /// Operators
    Position2dDTO& operator=(const Position2dDTO& o) Q_DECL_NOEXCEPT
    {
        _position_x = o._position_x;
        _position_y = o._position_y;

        return *this;
    }

    bool operator==(const Position2dDTO& o) const Q_DECL_NOEXCEPT
    {
        return _position_x == o._position_x && _position_y == o._position_y;
    }

    bool operator!=(const Position2dDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    float _position_x{0.0}; /// In meters
    float _position_y{0.0}; /// In meters
};

} // namespace gcs
