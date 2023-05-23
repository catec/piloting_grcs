#ifndef CURRENT_INSP_ITEM_DTO_H_
#define CURRENT_INSP_ITEM_DTO_H_

#include <common/IDTO.h>

namespace gcs {

class CurrentInspItemDTO : public IDTO
{
  public:
    virtual ~CurrentInspItemDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::CURRENT_INSP_ITEM; }

    qint16& getIndex() Q_DECL_NOEXCEPT { return _index; }

    const qint16& getIndex() const Q_DECL_NOEXCEPT { return _index; }

    /// Operators
    CurrentInspItemDTO& operator=(const CurrentInspItemDTO& o) Q_DECL_NOEXCEPT
    {
        _index = o._index;

        return *this;
    }

    bool operator==(const CurrentInspItemDTO& o) const Q_DECL_NOEXCEPT { return _index == o._index; }

    bool operator!=(const CurrentInspItemDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    qint16 _index{-1};
};

} // namespace gcs

#endif