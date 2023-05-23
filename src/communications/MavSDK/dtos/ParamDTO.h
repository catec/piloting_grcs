#pragma once

#include <common/IDTO.h>

namespace gcs {

class ParamDTO : public IDTO
{
  public:
    virtual ~ParamDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::PARAM; }

    qint16& getIndex() Q_DECL_NOEXCEPT { return _index; }

    const qint16& getIndex() const Q_DECL_NOEXCEPT { return _index; }

    float& getValue() Q_DECL_NOEXCEPT { return _value; }

    const float& getValue() const Q_DECL_NOEXCEPT { return _value; }

    QString& getLabel() Q_DECL_NOEXCEPT { return _label; }

    const QString& getLabel() const Q_DECL_NOEXCEPT { return _label; }

    QString& getDescription() Q_DECL_NOEXCEPT { return _description; }

    const QString& getDescription() const Q_DECL_NOEXCEPT { return _description; }

    QString& getUnits() Q_DECL_NOEXCEPT { return _units; }

    const QString& getUnits() const Q_DECL_NOEXCEPT { return _units; }
    /// Operators
    ParamDTO&      operator=(const ParamDTO& o) Q_DECL_NOEXCEPT
    {
        _index       = o._index;
        _label       = o._label;
        _description = o._description;
        _value       = o._value;
        _units       = o._units;

        return *this;
    }

    bool operator==(const ParamDTO& o) const Q_DECL_NOEXCEPT
    {
        return _index == o._index && _label == o._label && _description == o._description && _value == o._value
            && _units == o._units;
    }

    bool operator!=(const ParamDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    qint16  _index{-1};
    QString _label;
    QString _description;
    float   _value;
    QString _units;
};

} // namespace gcs
