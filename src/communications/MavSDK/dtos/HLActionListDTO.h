#ifndef HL_ACTION_LIST_DTO_H_
#define HL_ACTION_LIST_DTO_H_

#include <common/IDTO.h>

#include "HLActionItemDTO.h"

namespace gcs {

class HLActionListDTO : public IDTO
{
  public:
    virtual ~HLActionListDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::HLACTION_LIST; }

    QList<HLActionItemDTO>& getHLActionList() Q_DECL_NOEXCEPT { return _hlActionList; }

    const QList<HLActionItemDTO>& getHLActionList() const Q_DECL_NOEXCEPT { return _hlActionList; }

    /// Operators
    HLActionListDTO& operator=(const HLActionListDTO& o) Q_DECL_NOEXCEPT
    {
        _hlActionList = o._hlActionList;

        return *this;
    }

    bool operator==(const HLActionListDTO& o) const Q_DECL_NOEXCEPT { return _hlActionList == o._hlActionList; }

    bool operator!=(const HLActionListDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    QList<HLActionItemDTO> _hlActionList;
};

} // namespace gcs

#endif