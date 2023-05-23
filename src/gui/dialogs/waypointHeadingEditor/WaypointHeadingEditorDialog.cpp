#include "WaypointHeadingEditorDialog.h"

#include <QsLog/QsLog.h>
#include <common/Types.h>

#include <QShowEvent>
#include <QStyle>

#include "dataModel/dtos/CloudDimensionsDTO.h"
#include "dataModel/dtos/CloudEditionParametersDTO.h"
#include "ui_WaypointHeadingEditorDialog.h"

#define SCALE_FLOAT2INT 10.0

namespace gcs {

WaypointHeadingEditorDialog::WaypointHeadingEditorDialog(QWidget* parent) :
        QDialog(parent), _ui(std::make_unique<Ui::WaypointHeadingEditorDialog>())
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _ui->setupUi(this);

    this->setWindowFlags(this->windowFlags() | Qt::Tool);

    _ui->localOrientDial->setRange(-180 * SCALE_FLOAT2INT, 180 * SCALE_FLOAT2INT);
    _ui->localOrientDial->setNotchTarget(5);
    _ui->localOrientDial->setNotchesVisible(true);
    _ui->localOrientDial->setPageStep(20 * SCALE_FLOAT2INT);

    _ui->dialProgressB->setRange(-180, 180);
    _ui->dialProgressB->setFormat("%v deg");

    connect(_ui->localOrientDial, &QDial::valueChanged, [=](const int& newValue) {
        _ui->dialProgressB->setValue(static_cast<int>(newValue / SCALE_FLOAT2INT));
        _ui->headingValueSB->setValue(static_cast<float>(newValue / SCALE_FLOAT2INT));
    });

    connect(_ui->localOrientDial, &QDial::sliderReleased, this, &WaypointHeadingEditorDialog::sliderReleased);

    connect(_ui->headingValueSB,
            static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            [=](const double& value) {
                const auto valueInt = static_cast<int>(value * SCALE_FLOAT2INT);

                _ui->localOrientDial->setValue(valueInt);
                _ui->dialProgressB->setValue(valueInt);

                Q_EMIT modifyPreviewWp(value);
            });

    connect(_ui->applyRotationB, &QPushButton::clicked, this, &WaypointHeadingEditorDialog::applyButtonPressed);

    _ui->localOrientDial->setValue(0);
    _ui->dialProgressB->setValue(0.0);
}

WaypointHeadingEditorDialog::~WaypointHeadingEditorDialog()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

void WaypointHeadingEditorDialog::sliderReleased()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    Q_EMIT modifyPreviewWp(_ui->headingValueSB->value());
}

void WaypointHeadingEditorDialog::applyButtonPressed()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    Q_EMIT modifyCurrentWp(_ui->headingValueSB->value());

    _ui->localOrientDial->setValue(0);
    _ui->dialProgressB->setValue(0.0);
}

} // namespace gcs
