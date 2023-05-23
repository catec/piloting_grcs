#pragma once

#include <common/IDTO.h>
#include <common/Types.h>
#include <dataModel/Position3dDTO.h>
#include <dataModel/QuaternionDTO.h>
#include <dataModel/dtos/InspectionTypeDTO.h>

#include <QDateTime>

namespace gcs {

class InspectionTaskDTO : public IDTO
{
  public:
    virtual ~InspectionTaskDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::INSPECTION_TASK; }

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

    InspectionTaskLocationType& getLocationType() Q_DECL_NOEXCEPT { return _locationType; }

    const InspectionTaskLocationType& getLocationType() const Q_DECL_NOEXCEPT { return _locationType; }

    InspectionTypeDTO& getType() Q_DECL_NOEXCEPT { return _type; }

    const InspectionTypeDTO& getType() const Q_DECL_NOEXCEPT { return _type; }

    Position3dDTO& getPosition() Q_DECL_NOEXCEPT { return _position; }

    const Position3dDTO& getPosition() const Q_DECL_NOEXCEPT { return _position; }

    QuaternionDTO& getOrientation() Q_DECL_NOEXCEPT { return _quaternion; }

    const QuaternionDTO& getOrientation() const Q_DECL_NOEXCEPT { return _quaternion; }

    /// \note. Will be good idea create something like "DimensionsDTO" ?
    float& getWidth() Q_DECL_NOEXCEPT { return _width; }

    const float& getWidth() const Q_DECL_NOEXCEPT { return _width; }

    float& getHeight() Q_DECL_NOEXCEPT { return _height; }

    const float& getHeight() const Q_DECL_NOEXCEPT { return _height; }

    float& getDepth() Q_DECL_NOEXCEPT { return _depth; }

    const float& getDepth() const Q_DECL_NOEXCEPT { return _depth; }

    /// Operators
    InspectionTaskDTO& operator=(const InspectionTaskDTO& o) Q_DECL_NOEXCEPT
    {
        _name           = o._name;
        _uuid           = o._uuid;
        _description    = o._description;
        _creationDate   = o._creationDate;
        _lastUpdateDate = o._lastUpdateDate;
        _locationType   = o._locationType;
        _type           = o._type;
        _position       = o._position;
        _quaternion     = o._quaternion;
        _width          = o._width;
        _height         = o._height;
        _depth          = o._depth;

        return *this;
    }

    bool operator==(const InspectionTaskDTO& o) const Q_DECL_NOEXCEPT
    {
        return _name == o._name && _uuid == o._uuid && _description == o._description
            && _creationDate == o._creationDate && _lastUpdateDate == o._lastUpdateDate
            && _locationType == o._locationType && _type == o._type;
        _position == o._position&& _quaternion == o._quaternion&& _width == o._width&& _height == o._height&& _depth
                == o._depth;
    }

    bool operator!=(const InspectionTaskDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    QString                    _name;
    QString                    _uuid;
    QString                    _description;
    QDateTime                  _creationDate;
    QDateTime                  _lastUpdateDate;
    InspectionTaskLocationType _locationType;
    InspectionTypeDTO          _type;

    Position3dDTO _position;
    QuaternionDTO _quaternion;

    float _width;
    float _height;
    float _depth;
};

} // namespace gcs
