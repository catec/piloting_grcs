#ifndef CHECKLIST_ITEM_DTO_H_
#define CHECKLIST_ITEM_DTO_H_

#include <common/IDTO.h>
#include <common/Types.h>

namespace gcs {

class CheckListItemDTO : public IDTO
{
  public:
    virtual ~CheckListItemDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::CHECKLIST_ITEM; }

    quint16& getIndex() Q_DECL_NOEXCEPT { return _index; }

    const quint16& getIndex() const Q_DECL_NOEXCEPT { return _index; }

    QString& getName() Q_DECL_NOEXCEPT { return _name; }

    const QString& getName() const Q_DECL_NOEXCEPT { return _name; }

    QString& getDescription() Q_DECL_NOEXCEPT { return _description; }

    const QString& getDescription() const Q_DECL_NOEXCEPT { return _description; }

    CheckStatus& getCheckStatus() Q_DECL_NOEXCEPT { return _checkStatus; }

    const CheckStatus& getCheckStatus() const Q_DECL_NOEXCEPT { return _checkStatus; }

    /// Operators
    CheckListItemDTO& operator=(const CheckListItemDTO& o) Q_DECL_NOEXCEPT
    {
        _index       = o._index;
        _name        = o._name;
        _description = o._description;
        _checkStatus = o._checkStatus;

        return *this;
    }

    bool operator==(const CheckListItemDTO& o) const Q_DECL_NOEXCEPT
    {
        return _index == o._index && _name == o._name && _description == o._description
            && _checkStatus == o._checkStatus;
    }

    bool operator!=(const CheckListItemDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    quint16     _index;
    QString     _name;
    QString     _description;
    CheckStatus _checkStatus{CheckStatus::NONE};
};

} // namespace gcs

#endif