#ifndef COMMAND_BASIC_DTO_H_
#define COMMAND_BASIC_DTO_H_

#include <common/IDTO.h>
#include <common/Types.h>

namespace gcs {

class CommandBasicDTO : public IDTO
{
  public:
    virtual ~CommandBasicDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::COMMAND_BASIC; }

    quint16& getId() Q_DECL_NOEXCEPT { return _id; }

    const quint16& getId() const Q_DECL_NOEXCEPT { return _id; }

    QString& getName() Q_DECL_NOEXCEPT { return _name; }

    const QString& getName() const Q_DECL_NOEXCEPT { return _name; }

    QString& getDescription() Q_DECL_NOEXCEPT { return _description; }

    const QString& getDescription() const Q_DECL_NOEXCEPT { return _description; }

    quint8& getConfirmation() Q_DECL_NOEXCEPT { return _confirmation; }

    const quint8& getConfirmation() const Q_DECL_NOEXCEPT { return _confirmation; }

    /// Operators
    CommandBasicDTO& operator=(const CommandBasicDTO& o) Q_DECL_NOEXCEPT
    {
        _id           = o._id;
        _confirmation = o._confirmation;
        _name         = o._name;
        _description  = o._description;

        return *this;
    }

    bool operator==(const CommandBasicDTO& o) const Q_DECL_NOEXCEPT
    {
        return _id == o._id && _confirmation == o._confirmation && _name == o._name && _description == o._description;
    }

    bool operator!=(const CommandBasicDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    quint16 _id;
    quint8  _confirmation;
    QString _name;
    QString _description;
};

} // namespace gcs

#endif