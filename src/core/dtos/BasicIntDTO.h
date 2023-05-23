#pragma once

#include <common/IDTO.h>

namespace gcs {

class BasicIntDTO : public IDTO
{
  public:
    virtual ~BasicIntDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::BASIC_INT; }

    int& getInt() Q_DECL_NOEXCEPT { return _int; }

    const int& getInt() const Q_DECL_NOEXCEPT { return _int; }

    /// Operators
    BasicIntDTO& operator=(const BasicIntDTO& o) Q_DECL_NOEXCEPT
    {
        _int = o._int;

        return *this;
    }

    bool operator==(const BasicIntDTO& o) const Q_DECL_NOEXCEPT { return _int == o._int; }

    bool operator!=(const BasicIntDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    int _int{0};
};

} // namespace gcs
