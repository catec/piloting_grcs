#pragma once

#include <common/IDTO.h>

namespace gcs {

class CloudDimensionsDTO : public IDTO
{
  public:
    virtual ~CloudDimensionsDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::CLOUD_DIMENSIONS; }

    int& getNumPoints() Q_DECL_NOEXCEPT { return _numPoints; }

    const int& getNumPoints() const Q_DECL_NOEXCEPT { return _numPoints; }

    float& getXmin() Q_DECL_NOEXCEPT { return _xMin; }

    const float& getXmin() const Q_DECL_NOEXCEPT { return _xMin; }

    float& getYmin() Q_DECL_NOEXCEPT { return _yMin; }

    const float& getYmin() const Q_DECL_NOEXCEPT { return _yMin; }

    float& getZmin() Q_DECL_NOEXCEPT { return _zMin; }

    const float& getZmin() const Q_DECL_NOEXCEPT { return _zMin; }

    float& getXmax() Q_DECL_NOEXCEPT { return _xMax; }

    const float& getXmax() const Q_DECL_NOEXCEPT { return _xMax; }

    float& getYmax() Q_DECL_NOEXCEPT { return _yMax; }

    const float& getYmax() const Q_DECL_NOEXCEPT { return _yMax; }

    float& getZmax() Q_DECL_NOEXCEPT { return _zMax; }

    const float& getZmax() const Q_DECL_NOEXCEPT { return _zMax; }

    /// Operators
    CloudDimensionsDTO& operator=(const CloudDimensionsDTO& o) Q_DECL_NOEXCEPT
    {
        _numPoints = o._numPoints;
        _xMin      = o._xMin;
        _yMin      = o._yMin;
        _zMin      = o._zMin;
        _xMax      = o._xMax;
        _yMax      = o._yMax;
        _zMax      = o._zMax;

        return *this;
    }

    bool operator==(const CloudDimensionsDTO& o) const Q_DECL_NOEXCEPT
    {
        return _numPoints == o._numPoints && _xMin == o._xMin && _yMin == o._yMin && _zMin == o._zMin
            && _xMax == o._xMax && _yMax == o._yMax && _zMax == o._zMax;
    }

    bool operator!=(const CloudDimensionsDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    int _numPoints{0};

    float _xMin{0.0};
    float _yMin{0.0};
    float _zMin{0.0};

    float _xMax{0.0};
    float _yMax{0.0};
    float _zMax{0.0};
};
} // namespace gcs