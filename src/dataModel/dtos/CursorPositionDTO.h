#pragma once

#include <dataModel/Position3dDTO.h>

namespace gcs {

class CursorPositionDTO : public Position3dDTO
{
  public:
    virtual ~CursorPositionDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::CURSOR_POSITION; }

    /// Operators
    CursorPositionDTO& operator=(const CursorPositionDTO& o) Q_DECL_NOEXCEPT
    {
        Position3dDTO::operator=(o);

        return *this;
    }

    bool operator==(const CursorPositionDTO& o) const Q_DECL_NOEXCEPT { return Position3dDTO::operator==(o); }

    bool operator!=(const CursorPositionDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }
};

} // namespace gcs
