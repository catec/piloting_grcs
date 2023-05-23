#pragma once

#include <common/IDTO.h>

#include <QStringList>

namespace gcs {

class BasicStringListDTO : public IDTO
{
  public:
    virtual ~BasicStringListDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::BASIC_STRING_LIST; }

    QStringList& getStringList() Q_DECL_NOEXCEPT { return _stringList; }

    const QStringList& getStringList() const Q_DECL_NOEXCEPT { return _stringList; }

    /// Operators
    BasicStringListDTO& operator=(const BasicStringListDTO& o) Q_DECL_NOEXCEPT
    {
        _stringList = o._stringList;

        return *this;
    }

    bool operator==(const BasicStringListDTO& o) const Q_DECL_NOEXCEPT { return _stringList == o._stringList; }

    bool operator!=(const BasicStringListDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    QStringList _stringList;
};

} // namespace gcs
