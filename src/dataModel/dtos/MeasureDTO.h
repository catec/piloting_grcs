#pragma once

#include <dataModel/Position3dDTO.h>

#include <QList>

namespace gcs {

class MeasureDTO : public IDTO
{
  public:
    virtual ~MeasureDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::MEASURE; }

    QList<Position3dDTO>& getPointList() Q_DECL_NOEXCEPT { return _pointList; }

    const QList<Position3dDTO>& getPointList() const Q_DECL_NOEXCEPT { return _pointList; }

    /// Operators
    MeasureDTO& operator=(const MeasureDTO& o) Q_DECL_NOEXCEPT
    {
        _pointList = o._pointList;

        return *this;
    }

    bool operator==(const MeasureDTO& o) const Q_DECL_NOEXCEPT { return _pointList == o._pointList; }

    bool operator!=(const MeasureDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    QList<Position3dDTO> _pointList;
};

} // namespace gcs
