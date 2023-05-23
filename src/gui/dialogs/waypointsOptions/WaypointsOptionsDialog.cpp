#include "WaypointsOptionsDialog.h"

#include <QsLog/QsLog.h>
#include <dataModel/dtos/InspectionTaskDTO.h>

#include <QLineEdit>

#include "ui_WaypointsOptionsDialog.h"

namespace gcs {

WaypointsOptionsDialog::WaypointsOptionsDialog(QWidget* parent, bool showAltitude) :
        QDialog(parent), _ui(std::make_unique<Ui::WaypointsOptionsDialog>())
{
    _ui->setupUi(this);

    if (!showAltitude) {
        _ui->defaultAltitudeSB->setEnabled(false);
    }

    // clang-format off
    connect(_ui->taskIdCB,  QOverload<int>::of(&QComboBox::activated),
            this,           &WaypointsOptionsDialog::itemChanged);
    
    connect(_ui->nameCB,    QOverload<int>::of(&QComboBox::activated),
            this,           &WaypointsOptionsDialog::itemChanged);
    // clang-format on
}

WaypointsOptionsDialog::~WaypointsOptionsDialog() {}

void WaypointsOptionsDialog::setDefaultAltitude(const float& defaultAltitude)
{
    _ui->defaultAltitudeSB->setValue(defaultAltitude);
}

float WaypointsOptionsDialog::getDefaultAltitude() const
{
    return static_cast<float>(_ui->defaultAltitudeSB->value());
}

void WaypointsOptionsDialog::setInspectionTaskList(const QList<InspectionTaskDTO>& inspectionTaskList)
{
    if (_ui->taskIdCB->count() > 0) {
        _ui->taskIdCB->clear();
    }

    if (_ui->nameCB->count() > 0) {
        _ui->nameCB->clear();
    }

    _ui->taskIdCB->lineEdit()->setReadOnly(true);
    _ui->taskIdCB->lineEdit()->setAlignment(Qt::AlignCenter);

    _ui->nameCB->lineEdit()->setReadOnly(true);
    _ui->nameCB->lineEdit()->setAlignment(Qt::AlignCenter);

    for (int i = 0; i < inspectionTaskList.size(); i++) {
        _ui->taskIdCB->addItem(inspectionTaskList.at(i).getUUID());
        _ui->taskIdCB->setItemData(i, Qt::AlignCenter, Qt::TextAlignmentRole);
        _ui->nameCB->addItem(inspectionTaskList.at(i).getName());
        _ui->nameCB->setItemData(i, Qt::AlignCenter, Qt::TextAlignmentRole);
    }
}

QString WaypointsOptionsDialog::getInspectionTaskUUID() const
{
    return _ui->taskIdCB->currentText();
}

void WaypointsOptionsDialog::itemChanged(int index)
{
    auto comboB = dynamic_cast<QComboBox*>(QObject::sender());

    if (comboB == _ui->nameCB) {
        _ui->taskIdCB->setCurrentIndex(index);
    } else if (comboB == _ui->taskIdCB) {
        _ui->nameCB->setCurrentIndex(index);
    }
}

} // namespace gcs
