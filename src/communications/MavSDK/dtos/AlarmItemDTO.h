#ifndef ALARM_ITEM_DTO_H_
#define ALARM_ITEM_DTO_H_

#include <common/IDTO.h>

namespace gcs {

class AlarmItemDTO : public IDTO
{
  public:
    virtual ~AlarmItemDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::ALARM_ITEM; }

    quint16& getIndex() Q_DECL_NOEXCEPT { return _index; }

    const quint16& getIndex() const Q_DECL_NOEXCEPT { return _index; }

    QString& getName() Q_DECL_NOEXCEPT { return _name; }

    const QString& getName() const Q_DECL_NOEXCEPT { return _name; }

    QString& getDescription() Q_DECL_NOEXCEPT { return _description; }

    const QString& getDescription() const Q_DECL_NOEXCEPT { return _description; }
    /// Operators
    AlarmItemDTO&  operator=(const AlarmItemDTO& o) Q_DECL_NOEXCEPT
    {
        _index       = o._index;
        _name        = o._name;
        _description = o._description;

        return *this;
    }

    bool operator==(const AlarmItemDTO& o) const Q_DECL_NOEXCEPT
    {
        return _index == o._index && _name == o._name && _description == o._description;
    }

    bool operator!=(const AlarmItemDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    quint16 _index;
    QString _name;
    QString _description;
};

} // namespace gcs

#endif