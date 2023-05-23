
#include "VisualizationConfigDialog.h"

#include <QsLog/QsLog.h>

#include <QLineEdit>
#include <QVariant>

#include "ui_VisualizationConfigDialog.h"

#define SHOW      0
#define HIDE      1
#define RGB8      0
#define FLATCOLOR 1

namespace gcs {

VisualizationConfigDialog::VisualizationConfigDialog(QWidget* parent) :
        QDialog(parent), _ui(std::make_unique<Ui::VisualizationConfigDialog>())
{
    _ui->setupUi(this);

    setupComboBoxBoolean(_ui->showGridCB);
    setupComboBoxBoolean(_ui->showItemsTextCB);
    setupComboBoxBoolean(_ui->showGlobalAxisCB);
    setupComboBoxBoolean(_ui->showHomeCB);

    _ui->colorStyleCB->lineEdit()->setAlignment(Qt::AlignCenter);
    _ui->colorStyleCB->lineEdit()->setReadOnly(true);
    _ui->colorStyleCB->addItem("RGB8");
    _ui->colorStyleCB->addItem("FlatColor");
    _ui->colorStyleCB->setItemData(RGB8, Qt::AlignCenter, Qt::TextAlignmentRole);
    _ui->colorStyleCB->setItemData(FLATCOLOR, Qt::AlignCenter, Qt::TextAlignmentRole);
    _ui->flatColorB->setEnabled(false);

    // clang-format off
    connect(_ui->flatColorB,   &QPushButton::clicked,
            this,              &VisualizationConfigDialog::flatColorSelect);
    connect(_ui->colorStyleCB, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,              &VisualizationConfigDialog::colorStyleChanged);
    // clang-format on
}

VisualizationConfigDialog::~VisualizationConfigDialog() {}

void VisualizationConfigDialog::setupComboBoxBoolean(QComboBox* combobox)
{
    if (!combobox) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << "Invalid combobox";
        return;
    }

    combobox->lineEdit()->setAlignment(Qt::AlignCenter);
    combobox->lineEdit()->setReadOnly(true);
    combobox->addItem("SHOW");
    combobox->addItem("HIDE");
    combobox->setItemData(SHOW, Qt::AlignCenter, Qt::TextAlignmentRole);
    combobox->setItemData(HIDE, Qt::AlignCenter, Qt::TextAlignmentRole);
}

void VisualizationConfigDialog::setShowFloorGrid(const bool value)
{
    _ui->showGridCB->setCurrentIndex(value ? SHOW : HIDE);
}

bool VisualizationConfigDialog::getShowFloorGrid() const
{
    return _ui->showGridCB->currentIndex() == SHOW ? true : false;
}

void VisualizationConfigDialog::setShowItemsText(const bool value)
{
    _ui->showItemsTextCB->setCurrentIndex(value ? SHOW : HIDE);
}

bool VisualizationConfigDialog::getShowItemsText() const
{
    return _ui->showItemsTextCB->currentIndex() == SHOW ? true : false;
}

void VisualizationConfigDialog::setShowGlobalAxis(const bool value)
{
    _ui->showGlobalAxisCB->setCurrentIndex(value ? SHOW : HIDE);
}

bool VisualizationConfigDialog::getShowGlobalAxis() const
{
    return _ui->showGlobalAxisCB->currentIndex() == SHOW ? true : false;
}

void VisualizationConfigDialog::setShowHomePosition(const bool value)
{
    _ui->showHomeCB->setCurrentIndex(value ? SHOW : HIDE);
}

bool VisualizationConfigDialog::getShowHomePosition() const
{
    return _ui->showHomeCB->currentIndex() == SHOW ? true : false;
}

void VisualizationConfigDialog::setWaypointSizeScale(const float scale)
{
    _ui->wptsScaleSB->setValue(scale);
}

float VisualizationConfigDialog::getWaypointSizeScale() const
{
    return _ui->wptsScaleSB->value();
}

void VisualizationConfigDialog::setInspectionPtScale(const float scale)
{
    _ui->inspPtScaleSB->setValue(scale);
}

float VisualizationConfigDialog::getInspPtSizeScale() const
{
    return _ui->inspPtScaleSB->value();
}

void VisualizationConfigDialog::setInspLocationTransp(const float scale)
{
    _ui->inspLocTranspSB->setValue(scale);
}

float VisualizationConfigDialog::getInspLocationTransp() const
{
    return _ui->inspLocTranspSB->value();
}

void VisualizationConfigDialog::setCloudPtsSize(const float size)
{
    _ui->cloudPtsSizeSB->setValue(size);
}

float VisualizationConfigDialog::getCloudPtsSize() const
{
    return _ui->cloudPtsSizeSB->value();
}

void VisualizationConfigDialog::setPointCloudColorStyle(const QVariant& colorStyle)
{
    if (colorStyle.toString() == "FlatColor") {
        _ui->colorStyleCB->setCurrentIndex(FLATCOLOR);
    } else {
        _ui->colorStyleCB->setCurrentIndex(RGB8);
    }
}

QVariant VisualizationConfigDialog::getPointCloudColorStyle() const
{
    return _ui->colorStyleCB->currentIndex() == FLATCOLOR ? QVariant("FlatColor") : QVariant("RGB8");
}

void VisualizationConfigDialog::setPointCloudColor(const QColor& color)
{
    if (color.isValid()) {
        const auto qss = QString("background-color: %1").arg(color.name());
        _ui->flatColorB->setStyleSheet(qss);
        _currentColor = color;
    }
}

QColor VisualizationConfigDialog::getPointCloudColor() const
{
    return _currentColor;
}

void VisualizationConfigDialog::flatColorSelect()
{
    QColorDialog dialog(this);
    dialog.setOption(QColorDialog::DontUseNativeDialog);

    if (dialog.exec() != QDialog::Accepted) {
        return;
    }

    setPointCloudColor(dialog.currentColor());
}

void VisualizationConfigDialog::colorStyleChanged(int index)
{
    if (index == FLATCOLOR) {
        _ui->flatColorB->setEnabled(true);
    } else {
        _ui->flatColorB->setEnabled(false);
    }
}

} // namespace gcs
