#pragma once

#include <common/IDTO.h>
#include <common/Types.h>

namespace gcs {

class ViewModeDTO : public IDTO
{
  public:
    virtual ~ViewModeDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::VIEW_MODE; }

    ViewModeType& getMode() Q_DECL_NOEXCEPT { return _mode; }

    const ViewModeType& getMode() const Q_DECL_NOEXCEPT { return _mode; }

    /// Operators
    ViewModeDTO& operator=(const ViewModeDTO& o) Q_DECL_NOEXCEPT
    {
        _mode = o._mode;

        return *this;
    }

    bool operator==(const ViewModeDTO& o) const Q_DECL_NOEXCEPT { return _mode == o._mode; }

    bool operator!=(const ViewModeDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    ViewModeType _mode{ViewModeType::ThreeD};
};

} // namespace gcs
