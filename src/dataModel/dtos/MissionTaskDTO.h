#pragma once

#include <dataModel/dtos/InspectionTaskDTO.h>

#include <QList>

namespace gcs {

class MissionTaskDTO : public IDTO
{
  public:
    virtual ~MissionTaskDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::MISSION_TASK; }

    QString& getName() Q_DECL_NOEXCEPT { return _name; }

    const QString& getName() const Q_DECL_NOEXCEPT { return _name; }

    quint16& getId() Q_DECL_NOEXCEPT { return _id; }

    const quint16& getId() const Q_DECL_NOEXCEPT { return _id; }

    QString& getDescription() Q_DECL_NOEXCEPT { return _description; }

    const QString& getDescription() const Q_DECL_NOEXCEPT { return _description; }

    QDateTime& getCreationDateTime() Q_DECL_NOEXCEPT { return _creationDate; }

    const QDateTime& getCreationDateTime() const Q_DECL_NOEXCEPT { return _creationDate; }

    QDateTime& getLastUpdateDateTime() Q_DECL_NOEXCEPT { return _lastUpdateDate; }

    const QDateTime& getLastUpdateDateTime() const Q_DECL_NOEXCEPT { return _lastUpdateDate; }

    InspectionTaskDTO& getInspectionTaskAssociated() Q_DECL_NOEXCEPT { return _inspectionTaskAssociated; }

    const InspectionTaskDTO& getInspectionTaskAssociated() const Q_DECL_NOEXCEPT { return _inspectionTaskAssociated; }

    /// Operators
    MissionTaskDTO& operator=(const MissionTaskDTO& o) Q_DECL_NOEXCEPT
    {
        _name                     = o._name;
        _id                       = o._id;
        _description              = o._description;
        _creationDate             = o._creationDate;
        _inspectionTaskAssociated = o._inspectionTaskAssociated;
        _lastUpdateDate           = o._lastUpdateDate;

        return *this;
    }

    bool operator==(const MissionTaskDTO& o) const Q_DECL_NOEXCEPT
    {
        return _name == o._name && _id == o._id && _description == o._description && _creationDate == o._creationDate
            && _inspectionTaskAssociated == o._inspectionTaskAssociated && _lastUpdateDate == o._lastUpdateDate;
    }

    bool operator!=(const MissionTaskDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    QString           _name;
    quint16           _id{0};
    QString           _description;
    QDateTime         _creationDate;
    QDateTime         _lastUpdateDate;
    InspectionTaskDTO _inspectionTaskAssociated;
};

} // namespace gcs

Q_DECLARE_METATYPE(gcs::MissionTaskDTO)
Q_DECLARE_METATYPE(QList<gcs::MissionTaskDTO>)
