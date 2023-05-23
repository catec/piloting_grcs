#pragma once

#include <common/IDTO.h>

namespace gcs {

class EditionModeDTO : public IDTO
{
  public:
    virtual ~EditionModeDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::EDITION_MODE; }

    bool& getMode() Q_DECL_NOEXCEPT { return _mode; }

    const bool& getMode() const Q_DECL_NOEXCEPT { return _mode; }

    /// Operators
    EditionModeDTO& operator=(const EditionModeDTO& o) Q_DECL_NOEXCEPT
    {
        _mode = o._mode;

        return *this;
    }

    bool operator==(const EditionModeDTO& o) const Q_DECL_NOEXCEPT { return _mode == o._mode; }

    bool operator!=(const EditionModeDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    bool _mode{false};
};

} // namespace gcs
