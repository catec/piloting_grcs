#ifndef CHECKLIST_DTO_H_
#define CHECKLIST_DTO_H_

#include <common/IDTO.h>
#include <common/Types.h>

#include "CheckListItemDTO.h"

namespace gcs {

class CheckListDTO : public IDTO
{
  public:
    virtual ~CheckListDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::CHECKLIST; }

    QList<CheckListItemDTO>& getCheckItemsList() Q_DECL_NOEXCEPT { return _checklist; }

    const QList<CheckListItemDTO>& getCheckItemsList() const Q_DECL_NOEXCEPT { return _checklist; }

    /// Operators
    CheckListDTO& operator=(const CheckListDTO& o) Q_DECL_NOEXCEPT
    {
        _checklist = o._checklist;

        return *this;
    }

    bool operator==(const CheckListDTO& o) const Q_DECL_NOEXCEPT { return _checklist == o._checklist; }

    bool operator!=(const CheckListDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    QList<CheckListItemDTO> _checklist;
};

} // namespace gcs

#endif