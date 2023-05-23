#pragma once

#include <common/IDTO.h>

#include <QString>

namespace gcs {

class BasicMissionDTO : public IDTO
{
  public:
    virtual ~BasicMissionDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::BASIC_MISSION; }

    QString& getName() Q_DECL_NOEXCEPT { return _name; }

    const QString& getName() const Q_DECL_NOEXCEPT { return _name; }

    QString& getPath() Q_DECL_NOEXCEPT { return _path; }

    const QString& getPath() const Q_DECL_NOEXCEPT { return _path; }

    /// Operators
    BasicMissionDTO& operator=(const BasicMissionDTO& o) Q_DECL_NOEXCEPT
    {
        _name = o._name;
        _path = o._path;

        return *this;
    }

    bool operator==(const BasicMissionDTO& o) const Q_DECL_NOEXCEPT { return _name == o._name && _path == o._path; }

    bool operator!=(const BasicMissionDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    QString _name;
    QString _path;
};

} // namespace gcs
