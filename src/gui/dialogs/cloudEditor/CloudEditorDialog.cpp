#include "CloudEditorDialog.h"

#include <QsLog/QsLog.h>
#include <common/Types.h>

#include <QShowEvent>
#include <QStyle>

#include "dataModel/dtos/CloudDimensionsDTO.h"
#include "dataModel/dtos/CloudEditionParametersDTO.h"
#include "ui_CloudEditorDialog.h"

namespace gcs {

CloudEditorDialog::CloudEditorDialog(QWidget* parent) : QDialog(parent), _ui(std::make_unique<Ui::CloudEditorDialog>())
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _ui->setupUi(this);

    _ui->infoWidget->setVisible(true);
    _ui->line->setVisible(true);

    this->setWindowFlags(this->windowFlags() | Qt::Tool);

    initPlaneCutParameters();

    connect(_ui->restoreCloudB, &QPushButton::clicked, this, &CloudEditorDialog::restoreButtonClicked);
    connect(_ui->hideCloudB, &QPushButton::clicked, this, &CloudEditorDialog::hidePointCloudButtonClicked);
}

CloudEditorDialog::~CloudEditorDialog()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

void CloudEditorDialog::showEvent(QShowEvent* event)
{
    event->accept();

    /// \note. Show dialog centered on parent
    QRect parentRect(parentWidget()->mapToGlobal(QPoint(0, 0)), parentWidget()->size());
    move(QStyle::alignedRect(Qt::LeftToRight, Qt::AlignCenter, size(), parentRect).topLeft());

    sendCutParameters(false);
}

void CloudEditorDialog::hideEvent(QHideEvent* event)
{
    event->accept();

    Q_EMIT hideCutPlane();
}

void CloudEditorDialog::closeEvent(QCloseEvent* event)
{
    event->ignore();
    hide();
}

void CloudEditorDialog::initPlaneCutParameters()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _axisBtnGroup.addButton(_ui->xAxisRB, static_cast<int>(Axis::X));
    _axisBtnGroup.addButton(_ui->yAxisRB, static_cast<int>(Axis::Y));
    _axisBtnGroup.addButton(_ui->zAxisRB, static_cast<int>(Axis::Z));

    _directionBtnGroup.addButton(_ui->upDirRB, 0);
    _directionBtnGroup.addButton(_ui->downDirRB, 1);

    connect(&_axisBtnGroup, static_cast<void (QButtonGroup::*)(int)>(&QButtonGroup::buttonClicked), [=](int /*id*/) {
        updateSliderRange();
        _ui->currentValSB->setValue(0.0);
        _ui->cutValueSlider->setValue(0);

        sendCutParameters(false);
    });

    connect(&_directionBtnGroup,
            static_cast<void (QButtonGroup::*)(int)>(&QButtonGroup::buttonClicked),
            [=](int /*id*/) { sendCutParameters(true); });

    connect(_ui->cutValueSlider, &QSlider::valueChanged, this, &CloudEditorDialog::indicateChanges);

    connect(_ui->cutValueSlider, &QSlider::sliderReleased, this, &CloudEditorDialog::indicateReleased);
}

void CloudEditorDialog::updateStatus(const bool isEnabled, const bool restoreBisEnabled)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _ui->xAxisRB->setEnabled(isEnabled);
    _ui->yAxisRB->setEnabled(isEnabled);
    _ui->zAxisRB->setEnabled(isEnabled);

    _ui->upDirRB->setEnabled(isEnabled);
    _ui->downDirRB->setEnabled(isEnabled);

    _ui->cutValueSlider->setEnabled(isEnabled);
    _ui->currentValSB->setEnabled(isEnabled);

    _ui->restoreCloudB->setEnabled(restoreBisEnabled);
}

void CloudEditorDialog::updateDimensions(const QSharedPointer<CloudDimensionsDTO>& dto)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _ui->numPtsL->setText(QString::number(dto->getNumPoints()));

    _ui->xminLE->setText(QString::number(dto->getXmin()));
    _ui->yminLE->setText(QString::number(dto->getYmin()));
    _ui->zminLE->setText(QString::number(dto->getZmin()));

    _ui->xmaxLE->setText(QString::number(dto->getXmax()));
    _ui->ymaxLE->setText(QString::number(dto->getYmax()));
    _ui->zmaxLE->setText(QString::number(dto->getZmax()));

    if (!_axisRanges.isEmpty()) {
        _axisRanges.clear();
    }

    /// \note. Multiply to add a decimal to slider. When get value divide from 10
    _axisRanges.append(QPair<int, int>(dto->getXmin() * 10, dto->getXmax() * 10));
    _axisRanges.append(QPair<int, int>(dto->getYmin() * 10, dto->getYmax() * 10));
    _axisRanges.append(QPair<int, int>(dto->getZmin() * 10, dto->getZmax() * 10));

    if (!isVisible()) {
        return;
    }

    updateSliderRange();
    _ui->currentValSB->setValue(0.0);
    _ui->cutValueSlider->setValue(0);
}

void CloudEditorDialog::updateSliderRange()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (_axisRanges.empty()) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << "Cloud dimensions not defined yet";
        return;
    }

    const auto currentRange = _axisRanges[_axisBtnGroup.checkedId()];

    _ui->cutValueSlider->setRange(currentRange.first, currentRange.second);
    _ui->currentValSB->setRange(currentRange.first / 10.0, currentRange.second / 10.0);
}

void CloudEditorDialog::indicateChanges(int currentValue)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    const float currentVal = currentValue / 10.0;
    _ui->currentValSB->setValue(currentVal);

    sendCutParameters(false);
}

void CloudEditorDialog::indicateReleased()
{
    sendCutParameters(true);
}

void CloudEditorDialog::restoreButtonClicked()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    updateStatus(true, true);
    Q_EMIT restorePointCloud();
}

void CloudEditorDialog::hidePointCloudButtonClicked()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    updateStatus(false, true);
    Q_EMIT hidePointCloud();
}

void CloudEditorDialog::sendCutParameters(const bool& apply_cut)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (_axisRanges.isEmpty()) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << "Cloud not created yet";
        return;
    }

    const bool isPositiveDir = (_directionBtnGroup.checkedId() == 0) ? true : false;

    auto editionParams                    = createSharedDTO<CloudEditionParametersDTO>();
    editionParams->getPlaneCutAxis()      = static_cast<Axis>(_axisBtnGroup.checkedId());
    editionParams->getPlaneCutDirection() = isPositiveDir;
    editionParams->getPlaneCutValue()     = _ui->currentValSB->value();
    editionParams->getApplyCut()          = apply_cut;

    Q_EMIT editPointCloud(editionParams);
}
} // namespace gcs
