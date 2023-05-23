#pragma once

#include "MsgConsoleDTO.h"

namespace gcs {

class MsgConsolePopupDTO : public MsgConsoleDTO
{
  public:
    virtual ~MsgConsolePopupDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::MSG_CONSOLE_POPUP; }

    QString& getTitle() Q_DECL_NOEXCEPT { return _title; }

    const QString& getTitle() const Q_DECL_NOEXCEPT { return _title; }

    /// Operators
    MsgConsolePopupDTO& operator=(const MsgConsolePopupDTO& o) Q_DECL_NOEXCEPT
    {
        MsgConsoleDTO::operator=(o);
        _title = o._title;

        return *this;
    }

    bool operator==(const MsgConsolePopupDTO& o) const Q_DECL_NOEXCEPT
    {
        return MsgConsoleDTO::operator==(o) && _title == o._title;
    }

    bool operator!=(const MsgConsolePopupDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    QString _title;
};

} // namespace gcs
