#ifndef REACHED_INSP_ITEM_DTO_H_
#define REACHED_INSP_ITEM_DTO_H_

#include <common/IDTO.h>

namespace gcs {

class ReachedInspItemDTO : public IDTO
{
  public:
    virtual ~ReachedInspItemDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::REACHED_INSP_ITEM; }

    qint16& getIndex() Q_DECL_NOEXCEPT { return _index; }

    const qint16& getIndex() const Q_DECL_NOEXCEPT { return _index; }

    /// Operators
    ReachedInspItemDTO& operator=(const ReachedInspItemDTO& o) Q_DECL_NOEXCEPT
    {
        _index = o._index;

        return *this;
    }

    bool operator==(const ReachedInspItemDTO& o) const Q_DECL_NOEXCEPT { return _index == o._index; }

    bool operator!=(const ReachedInspItemDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    qint16 _index{-1};
};

} // namespace gcs

#endif