#pragma once

#include <common/IDTO.h>
#include <common/Types.h>

namespace gcs {

class DialogDTO : public IDTO
{
  public:
    virtual ~DialogDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::DIALOG; }

    DialogType& getDialogType() Q_DECL_NOEXCEPT { return _dialogType; }

    const DialogType& getDialogType() const Q_DECL_NOEXCEPT { return _dialogType; }

    /// Operators
    DialogDTO& operator=(const DialogDTO& o) Q_DECL_NOEXCEPT
    {
        _dialogType = o._dialogType;

        return *this;
    }

    bool operator==(const DialogDTO& o) const Q_DECL_NOEXCEPT { return _dialogType == o._dialogType; }

    bool operator!=(const DialogDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    DialogType _dialogType{DialogType::Unknown};
};

} // namespace gcs
