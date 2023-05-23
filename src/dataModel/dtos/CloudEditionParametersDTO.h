#pragma once

#include <common/IDTO.h>
#include <common/Types.h>

namespace gcs {

class CloudEditionParametersDTO : public IDTO
{
  public:
    virtual ~CloudEditionParametersDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::CLOUD_EDITION_PARAMS; }

    bool& getApplyCut() Q_DECL_NOEXCEPT { return _applyCut; }

    const bool& getApplyCut() const Q_DECL_NOEXCEPT { return _applyCut; }

    Axis& getPlaneCutAxis() Q_DECL_NOEXCEPT { return _planeCutAxis; }

    const Axis& getPlaneCutAxis() const Q_DECL_NOEXCEPT { return _planeCutAxis; }

    bool& getPlaneCutDirection() Q_DECL_NOEXCEPT { return _planeCutDirection; }

    const bool& getPlaneCutDirection() const Q_DECL_NOEXCEPT { return _planeCutDirection; }

    float& getPlaneCutValue() Q_DECL_NOEXCEPT { return _planeCutValue; }

    const float& getPlaneCutValue() const Q_DECL_NOEXCEPT { return _planeCutValue; }

    /// Operators
    CloudEditionParametersDTO& operator=(const CloudEditionParametersDTO& o) Q_DECL_NOEXCEPT
    {
        _applyCut          = o._applyCut;
        _planeCutAxis      = o._planeCutAxis;
        _planeCutDirection = o._planeCutDirection;
        _planeCutValue     = o._planeCutValue;

        return *this;
    }

    bool operator==(const CloudEditionParametersDTO& o) const Q_DECL_NOEXCEPT
    {
        return _applyCut == o._applyCut && _planeCutAxis == o._planeCutAxis
            && _planeCutDirection == o._planeCutDirection && _planeCutValue == o._planeCutValue;
    }

    bool operator!=(const CloudEditionParametersDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    bool _applyCut{false};

    Axis _planeCutAxis{Axis::Z};
    bool _planeCutDirection{true}; // true: positive | false: negative

    float _planeCutValue{0.0};
};
} // namespace gcs