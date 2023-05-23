
#include "HomeParametersDialog.h"

#include "common/CommonDefines.h"
#include "dataModel/dtos/HomeDTO.h"
#include "ui_HomeParametersDialog.h"

namespace gcs {
HomeParametersDialog::HomeParametersDialog(QWidget* parent) :
        QDialog(parent), _ui(std::make_unique<Ui::HomeParametersDialog>())
{
    _ui->setupUi(this);
}

HomeParametersDialog::~HomeParametersDialog() {}

void HomeParametersDialog::setHomeParameters(const HomeDTO& homeDTO)
{
    _ui->positionxSB->setValue(homeDTO.getPosition().getX());
    _ui->positionySB->setValue(homeDTO.getPosition().getY());
    _ui->altitudeSB->setValue(homeDTO.getPosition().getZ());
    _ui->yawSB->setValue(homeDTO.getYawOrientation());
}

float HomeParametersDialog::getPositionX() const
{
    return static_cast<float>(_ui->positionxSB->value());
}

float HomeParametersDialog::getPositionY() const
{
    return static_cast<float>(_ui->positionySB->value());
}

float HomeParametersDialog::getPositionZ() const
{
    return static_cast<float>(_ui->altitudeSB->value());
}

float HomeParametersDialog::getYawAngle() const
{
    return static_cast<float>(_ui->yawSB->value());
}

} // namespace gcs
