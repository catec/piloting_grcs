
#pragma once

#include <common/IDTO.h>

#include <QDateTime>
#include <QtGlobal>

namespace gcs {

class InspectionTypeDTO : public IDTO
{
  public:
    virtual ~InspectionTypeDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::INSPECTION_TYPE; }

    QString& getUUID() Q_DECL_NOEXCEPT { return _uuid; }

    const QString& getUUID() const Q_DECL_NOEXCEPT { return _uuid; }

    QString& getName() Q_DECL_NOEXCEPT { return _name; }

    const QString& getName() const Q_DECL_NOEXCEPT { return _name; }

    QString& getDescription() Q_DECL_NOEXCEPT { return _description; }

    const QString& getDescription() const Q_DECL_NOEXCEPT { return _description; }

    QString& getProperties() Q_DECL_NOEXCEPT { return _properties; }

    const QString& getProperties() const Q_DECL_NOEXCEPT { return _properties; }

    QDateTime& getCreationDateTime() Q_DECL_NOEXCEPT { return _creationDateTime; }

    const QDateTime& getCreationDateTime() const Q_DECL_NOEXCEPT { return _creationDateTime; }

    QDateTime& getLastUpdateDateTime() Q_DECL_NOEXCEPT { return _lastUpdateDateTime; }

    const QDateTime&   getLastUpdateDateTime() const Q_DECL_NOEXCEPT { return _lastUpdateDateTime; }
    /// Operators
    InspectionTypeDTO& operator=(const InspectionTypeDTO& o) Q_DECL_NOEXCEPT
    {
        _uuid               = o._uuid;
        _name               = o._name;
        _description        = o._description;
        _properties         = o._properties;
        _creationDateTime   = o._creationDateTime;
        _lastUpdateDateTime = o._lastUpdateDateTime;

        return *this;
    }

    bool operator==(const InspectionTypeDTO& o) const Q_DECL_NOEXCEPT
    {
        return _uuid == o._uuid && _name == o._name && _description == o._description && _properties == o._properties
            && _creationDateTime == o._creationDateTime && _lastUpdateDateTime == o._lastUpdateDateTime;
    }

    bool operator!=(const InspectionTypeDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    QString   _uuid;
    QString   _name;
    QString   _description;
    QString   _properties;
    QDateTime _creationDateTime;
    QDateTime _lastUpdateDateTime;
};

} // namespace gcs
