#pragma once

#include <common/IDTO.h>

namespace gcs {

class BatteryParametersDTO : public IDTO
{
  public:
    virtual ~BatteryParametersDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::BATTERY_PARAMETERS; }

    float& getLowVoltLimit() Q_DECL_NOEXCEPT { return _lowVoltLimit; }

    const float& getLowVoltLimit() const Q_DECL_NOEXCEPT { return _lowVoltLimit; }

    float& getCritVoltLimit() Q_DECL_NOEXCEPT { return _critVoltLimit; }

    const float& getCritVoltLimit() const Q_DECL_NOEXCEPT { return _critVoltLimit; }

    /// Operators
    BatteryParametersDTO& operator=(const BatteryParametersDTO& o) Q_DECL_NOEXCEPT
    {
        _lowVoltLimit  = o._lowVoltLimit;
        _critVoltLimit = o._critVoltLimit;

        return *this;
    }

    bool operator==(const BatteryParametersDTO& o) const Q_DECL_NOEXCEPT
    {
        return _lowVoltLimit == o._lowVoltLimit && _critVoltLimit == o._critVoltLimit;
    }

    bool operator!=(const BatteryParametersDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    float _lowVoltLimit{0.0};  /// In volts
    float _critVoltLimit{0.0}; /// In volts
};

} // namespace gcs
