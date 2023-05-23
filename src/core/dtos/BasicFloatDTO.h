#pragma once

#include <common/IDTO.h>

namespace gcs {

class BasicFloatDTO : public IDTO
{
  public:
    virtual ~BasicFloatDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::BASIC_FLOAT; }

    float& getFloat() Q_DECL_NOEXCEPT { return _float; }

    const float& getFloat() const Q_DECL_NOEXCEPT { return _float; }

    /// Operators
    BasicFloatDTO& operator=(const BasicFloatDTO& o) Q_DECL_NOEXCEPT
    {
        _float = o._float;

        return *this;
    }

    bool operator==(const BasicFloatDTO& o) const Q_DECL_NOEXCEPT { return _float == o._float; }

    bool operator!=(const BasicFloatDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    float _float{0.0};
};

} // namespace gcs
