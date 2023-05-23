
#pragma once

#include <common/IDTO.h>

namespace gcs {

class ActionParametersDTO : public IDTO
{
  public:
    virtual ~ActionParametersDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::ACTION_PARAMETERS; }

    float& getParam1() Q_DECL_NOEXCEPT { return _param_1; }

    const float& getParam1() const Q_DECL_NOEXCEPT { return _param_1; }

    float& getParam2() Q_DECL_NOEXCEPT { return _param_2; }

    const float& getParam2() const Q_DECL_NOEXCEPT { return _param_2; }

    float& getParam3() Q_DECL_NOEXCEPT { return _param_3; }

    const float& getParam3() const Q_DECL_NOEXCEPT { return _param_3; }

    float& getParam4() Q_DECL_NOEXCEPT { return _param_4; }

    const float& getParam4() const Q_DECL_NOEXCEPT { return _param_4; }

    /// Operators
    ActionParametersDTO& operator=(const ActionParametersDTO& o) Q_DECL_NOEXCEPT
    {
        _param_1 = o._param_1;
        _param_2 = o._param_2;
        _param_3 = o._param_3;
        _param_4 = o._param_4;

        return *this;
    }

    bool operator==(const ActionParametersDTO& o) const Q_DECL_NOEXCEPT
    {
        return _param_1 == o._param_1 && _param_2 == o._param_2 && _param_3 == o._param_3 && _param_4 == o._param_4;
    }

    bool operator!=(const ActionParametersDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    float _param_1{0.0};
    float _param_2{0.0};
    float _param_3{0.0};
    float _param_4{0.0};
};

} // namespace gcs
