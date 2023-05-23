#pragma once

#include <common/IDTO.h>

namespace gcs {

class BasicBoolDTO : public IDTO
{
  public:
    virtual ~BasicBoolDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::BASIC_BOOL; }

    bool& getBool() Q_DECL_NOEXCEPT { return _bool; }

    const bool& getBool() const Q_DECL_NOEXCEPT { return _bool; }

    /// Operators
    BasicBoolDTO& operator=(const BasicBoolDTO& o) Q_DECL_NOEXCEPT
    {
        _bool = o._bool;

        return *this;
    }

    bool operator==(const BasicBoolDTO& o) const Q_DECL_NOEXCEPT { return _bool == o._bool; }

    bool operator!=(const BasicBoolDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    bool _bool{false};
};

} // namespace gcs
