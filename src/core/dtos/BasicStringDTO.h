#pragma once

#include <common/IDTO.h>

#include <QString>

namespace gcs {

class BasicStringDTO : public IDTO
{
  public:
    virtual ~BasicStringDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::BASIC_STRING; }

    QString& getString() Q_DECL_NOEXCEPT { return _string; }

    const QString& getString() const Q_DECL_NOEXCEPT { return _string; }

    /// Operators
    BasicStringDTO& operator=(const BasicStringDTO& o) Q_DECL_NOEXCEPT
    {
        _string = o._string;

        return *this;
    }

    bool operator==(const BasicStringDTO& o) const Q_DECL_NOEXCEPT { return _string == o._string; }

    bool operator!=(const BasicStringDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    QString _string;
};

} // namespace gcs
