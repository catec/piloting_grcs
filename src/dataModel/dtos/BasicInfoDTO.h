
#pragma once

#include <common/IDTO.h>

#include <QDateTime>
#include <QDebug>
#include <QString>

#include "InspectionTaskDTO.h"

namespace gcs {

class BasicInfoDTO : public IDTO
{
  public:
    virtual ~BasicInfoDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::BASIC_INSPECTION_PLAN; }

    QString& getName() Q_DECL_NOEXCEPT { return _name; }

    const QString& getName() const Q_DECL_NOEXCEPT { return _name; }

    QString& getUUID() Q_DECL_NOEXCEPT { return _uuid; }

    const QString& getUUID() const Q_DECL_NOEXCEPT { return _uuid; }

    QString& getDescription() Q_DECL_NOEXCEPT { return _description; }

    const QString& getDescription() const Q_DECL_NOEXCEPT { return _description; }

    QDateTime& getCreationDateTime() Q_DECL_NOEXCEPT { return _creationDate; }

    const QDateTime& getCreationDateTime() const Q_DECL_NOEXCEPT { return _creationDate; }

    QDateTime& getLastUpdateDateTime() Q_DECL_NOEXCEPT { return _lastUpdateDate; }

    const QDateTime& getLastUpdateDateTime() const Q_DECL_NOEXCEPT { return _lastUpdateDate; }

    /// Operators
    BasicInfoDTO& operator=(const BasicInfoDTO& o) Q_DECL_NOEXCEPT
    {
        _name           = o._name;
        _uuid           = o._uuid;
        _description    = o._description;
        _creationDate   = o._creationDate;
        _lastUpdateDate = o._lastUpdateDate;

        return *this;
    }

    bool operator==(const BasicInfoDTO& o) const Q_DECL_NOEXCEPT
    {
        return _name == o._name && _uuid == o._uuid && _description == o._description
            && _creationDate == o._creationDate && _lastUpdateDate == o._lastUpdateDate;
    }

    bool operator!=(const BasicInfoDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    QString _name;
    QString _uuid;
    QString _description;

    QDateTime _creationDate;
    QDateTime _lastUpdateDate;
};

} // namespace gcs
