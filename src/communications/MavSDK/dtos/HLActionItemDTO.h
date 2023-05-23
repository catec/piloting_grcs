#ifndef HL_ACTION_DTO_H_
#define HL_ACTION_DTO_H_

#include <common/IDTO.h>
#include <communications/MavSDK/dtos/CommandLongDTO.h>

namespace gcs {

class HLActionItemDTO : public IDTO
{
  public:
    virtual ~HLActionItemDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::HLACTION_ITEM; }

    quint16& getIndex() Q_DECL_NOEXCEPT { return _index; }

    const quint16& getIndex() const Q_DECL_NOEXCEPT { return _index; }

    QString& getName() Q_DECL_NOEXCEPT { return _name; }

    const QString& getName() const Q_DECL_NOEXCEPT { return _name; }

    QString& getDescription() Q_DECL_NOEXCEPT { return _description; }

    const QString& getDescription() const Q_DECL_NOEXCEPT { return _description; }

    CommandLongDTO& getAssociatedCommand() Q_DECL_NOEXCEPT { return _associated_command; }

    const CommandLongDTO& getAssociatedCommand() const Q_DECL_NOEXCEPT { return _associated_command; }
    /// Operators
    HLActionItemDTO&      operator=(const HLActionItemDTO& o) Q_DECL_NOEXCEPT
    {
        _index              = o._index;
        _name               = o._name;
        _description        = o._description;
        _command            = o._command;
        _associated_command = o._associated_command;

        return *this;
    }

    bool operator==(const HLActionItemDTO& o) const Q_DECL_NOEXCEPT
    {
        return _index == o._index && _name == o._name && _description == o._description && _command == o._command
            && _associated_command == o._associated_command;
    }

    bool operator!=(const HLActionItemDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    quint16        _index;
    QString        _name;
    QString        _description;
    quint16        _command;
    CommandLongDTO _associated_command;
};

} // namespace gcs

#endif