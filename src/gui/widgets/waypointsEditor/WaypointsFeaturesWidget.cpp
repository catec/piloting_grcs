
#include "WaypointsFeaturesWidget.h"

#include <QsLog/QsLog.h>
#include <common/CommonDefines.h>

#include <QLineEdit>

#include "dataModel/dtos/InspectionTaskDTO.h"
#include "dataModel/dtos/MissionDTO.h"
#include "ui_WaypointsFeaturesWidget.h"

#define YES 0
#define NO  1

namespace gcs {

WaypointsFeaturesWidget::WaypointsFeaturesWidget(QWidget* parent) :
        QWidget(parent), _ui(std::make_unique<Ui::WaypointsFeaturesWidget>())
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _ui->setupUi(this);

    createComboBoxes();

    clearFeatures();

    setConnections();

    waypointTypeChanged(WaypointType::POSE);
}

WaypointsFeaturesWidget::~WaypointsFeaturesWidget()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

void WaypointsFeaturesWidget::createComboBoxes()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _ui->typeLE->setReadOnly(true);

    _ui->autocontCB->lineEdit()->setAlignment(Qt::AlignCenter);
    _ui->autocontCB->lineEdit()->setReadOnly(true);
    _ui->autocontCB->addItem("YES");
    _ui->autocontCB->addItem("NO");
    _ui->autocontCB->setItemData(0, Qt::AlignCenter, Qt::TextAlignmentRole);
    _ui->autocontCB->setItemData(1, Qt::AlignCenter, Qt::TextAlignmentRole);
}

void WaypointsFeaturesWidget::setConnections()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    // clang-format off
   connect(_ui->autocontCB,    SIGNAL(currentIndexChanged(int)),
           this,               SLOT(indicateChanges()));
   connect(_ui->taskUuidCB,    SIGNAL(currentIndexChanged(int)),
           this,               SLOT(indicateChanges()));
   connect(_ui->posXSB,        SIGNAL(valueChanged(double)),
            this,              SLOT(indicateChanges()));
   connect(_ui->posYSB,        SIGNAL(valueChanged(double)),
            this,              SLOT(indicateChanges()));
   connect(_ui->posZSB,        SIGNAL(valueChanged(double)),
            this,              SLOT(indicateChanges()));
   connect(_ui->rollSB,        SIGNAL(valueChanged(double)),
            this,              SLOT(indicateChanges()));
   connect(_ui->pitchSB,       SIGNAL(valueChanged(double)),
            this,              SLOT(indicateChanges()));
   connect(_ui->yawSB,         SIGNAL(valueChanged(double)),
            this,              SLOT(indicateChanges()));
   connect(_ui->param1SB,      SIGNAL(valueChanged(double)),
            this,              SLOT(indicateChanges()));
   connect(_ui->param2SB,      SIGNAL(valueChanged(double)),
            this,              SLOT(indicateChanges()));
   connect(_ui->param3SB,      SIGNAL(valueChanged(double)),
            this,              SLOT(indicateChanges()));
   connect(_ui->param4SB,      SIGNAL(valueChanged(double)),
            this,              SLOT(indicateChanges()));

   connect(_ui->applyChangesB, &QPushButton::clicked,
            this,              &WaypointsFeaturesWidget::editWaypoint);
    // clang-format on
}

void WaypointsFeaturesWidget::updateInspectionTaskList(const QList<InspectionTaskDTO> inspectionTaskList)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (_ui->taskUuidCB->count() > 0) {
        _ui->taskUuidCB->clear();
    }

    _ui->taskUuidCB->lineEdit()->setAlignment(Qt::AlignCenter);
    _ui->taskUuidCB->lineEdit()->setReadOnly(true);

    for (int i = 0; i < inspectionTaskList.size(); i++) {
        _ui->taskUuidCB->addItem(inspectionTaskList.at(i).getUUID());
        _ui->taskUuidCB->setItemData(i, Qt::AlignCenter, Qt::TextAlignmentRole);
    }
}

void WaypointsFeaturesWidget::updateFeatures(const MissionDTO& mission)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    blockChanges(true);

    clearFeatures();
    waypointTypeChanged(WaypointType::POSE);

    for (const auto& route : mission.getLocalRoutesMap().values()) {
        for (const auto& waypoint : route.getWpsList()) {
            if (waypoint.getId() == mission.getCurrentWaypointId()) {
                const quint16 seqNumber = route.getWpsList().indexOf(waypoint) + 1;

                _ui->identifierSB->setValue(waypoint.getId());
                _ui->sequenceSB->setValue(seqNumber);

                const int task_index = _ui->taskUuidCB->findText(waypoint.getTaskUUID());
                if (task_index != -1) {
                    _ui->taskUuidCB->setCurrentIndex(task_index);
                } else {
                    QLOG_ERROR() << __PRETTY_FUNCTION__ << "Index not found in ComboBox";
                }

                _ui->autocontCB->setCurrentIndex(waypoint.getAutoContinue() ? YES : NO);

                _ui->posXSB->setValue(waypoint.getPosition().getX());
                _ui->posYSB->setValue(waypoint.getPosition().getY());
                _ui->posZSB->setValue(waypoint.getPosition().getZ());
                _ui->typeLE->setText(ToString(waypoint.getWpType()));

                const auto eaDeg = ToEulerAnglesDeg(
                        waypoint.getOrientation().getW(),
                        waypoint.getOrientation().getX(),
                        waypoint.getOrientation().getY(),
                        waypoint.getOrientation().getZ());
                _ui->rollSB->setValue(eaDeg[0]);
                _ui->pitchSB->setValue(eaDeg[1]);
                _ui->yawSB->setValue(eaDeg[2]);
                _ui->param1SB->setValue(waypoint.getActionParameters().getParam1());
                _ui->param2SB->setValue(waypoint.getActionParameters().getParam2());
                _ui->param3SB->setValue(waypoint.getActionParameters().getParam3());
                _ui->param4SB->setValue(waypoint.getActionParameters().getParam4());

                waypointTypeChanged(waypoint.getWpType());
            }
        }
    }

    blockChanges(false);
}

void WaypointsFeaturesWidget::waypointTypeChanged(const WaypointType& wpType)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    switch (wpType) {
        case WaypointType::POSE: {
            _ui->rollSB->setVisible(true);
            _ui->pitchSB->setVisible(true);
            _ui->yawSB->setVisible(true);
            _ui->rollL->setVisible(true);
            _ui->pitchL->setVisible(true);
            _ui->yawL->setVisible(true);

            _ui->posXSB->setReadOnly(false);
            _ui->posYSB->setReadOnly(false);
            _ui->posZSB->setReadOnly(false);

            _ui->param1SB->setVisible(false);
            _ui->param2SB->setVisible(false);
            _ui->param3SB->setVisible(false);
            _ui->param4SB->setVisible(false);
            _ui->param1L->setVisible(false);
            _ui->param2L->setVisible(false);
            _ui->param3L->setVisible(false);
            _ui->param4L->setVisible(false);

            break;
        }
        case WaypointType::ACTION: {
            _ui->rollSB->setVisible(false);
            _ui->pitchSB->setVisible(false);
            _ui->yawSB->setVisible(false);
            _ui->rollL->setVisible(false);
            _ui->pitchL->setVisible(false);
            _ui->yawL->setVisible(false);

            _ui->posXSB->setReadOnly(true);
            _ui->posYSB->setReadOnly(true);
            _ui->posZSB->setReadOnly(true);

            _ui->param1SB->setVisible(true);
            _ui->param2SB->setVisible(true);
            _ui->param3SB->setVisible(true);
            _ui->param4SB->setVisible(true);
            _ui->param1L->setVisible(true);
            _ui->param2L->setVisible(true);
            _ui->param3L->setVisible(true);
            _ui->param4L->setVisible(true);

            break;
        }
        case WaypointType::UNKNOWN:
        default: {
            break;
        }
    }
}

void WaypointsFeaturesWidget::setEnableFeatures(const bool value)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _ui->identifierSB->setEnabled(value);
    _ui->autocontCB->setEnabled(value);
    _ui->taskUuidCB->setEnabled(value);
    _ui->typeLE->setEnabled(value);
    _ui->posXSB->setEnabled(value);
    _ui->posYSB->setEnabled(value);
    _ui->posZSB->setEnabled(value);
    _ui->rollSB->setEnabled(value);
    _ui->pitchSB->setEnabled(value);
    _ui->yawSB->setEnabled(value);
    _ui->param1SB->setEnabled(value);
    _ui->param2SB->setEnabled(value);
    _ui->param3SB->setEnabled(value);
    _ui->param4SB->setEnabled(value);

    _ui->applyChangesB->setEnabled(value && _hasChanges);
}

void WaypointsFeaturesWidget::indicateChanges()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _hasChanges = true;
    _ui->applyChangesB->setEnabled(_hasChanges);
}

void WaypointsFeaturesWidget::editWaypoint()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    Position3dDTO position;
    position.getX() = static_cast<float>(_ui->posXSB->value());
    position.getY() = static_cast<float>(_ui->posYSB->value());
    position.getZ() = static_cast<float>(_ui->posZSB->value());

    auto wpType = ToWpType(_ui->typeLE->text().toStdString());

    auto          waypoint = createSharedDTO<WaypointDTO>();
    QuaternionDTO orientation;
    const auto    wp_quat = degToQuaternion(
            static_cast<float>(_ui->rollSB->value()),
            static_cast<float>(_ui->pitchSB->value()),
            static_cast<float>(_ui->yawSB->value()));
    orientation.getW() = wp_quat[0];
    orientation.getX() = wp_quat[1];
    orientation.getY() = wp_quat[2];
    orientation.getZ() = wp_quat[3];

    waypoint->getOrientation()                  = orientation;
    waypoint->getPosition()                     = position;
    waypoint->getActionParameters().getParam1() = _ui->param1SB->value();
    waypoint->getActionParameters().getParam2() = _ui->param2SB->value();
    waypoint->getActionParameters().getParam3() = _ui->param3SB->value();
    waypoint->getActionParameters().getParam4() = _ui->param4SB->value();

    waypoint->getWpType()       = wpType;
    waypoint->getId()           = _ui->identifierSB->value();
    waypoint->getAutoContinue() = _ui->autocontCB->currentIndex() == YES ? true : false;
    waypoint->getTaskUUID()     = _ui->taskUuidCB->currentText();

    Q_EMIT editWaypointFeatures(waypoint);
}

void WaypointsFeaturesWidget::blockChanges(const bool value)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _ui->identifierSB->blockSignals(value);
    _ui->autocontCB->blockSignals(value);
    _ui->taskUuidCB->blockSignals(value);
    _ui->typeLE->blockSignals(value);
    _ui->posXSB->blockSignals(value);
    _ui->posYSB->blockSignals(value);
    _ui->posZSB->blockSignals(value);
    _ui->rollSB->blockSignals(value);
    _ui->pitchSB->blockSignals(value);
    _ui->yawSB->blockSignals(value);
    _ui->param1SB->blockSignals(value);
    _ui->param2SB->blockSignals(value);
    _ui->param3SB->blockSignals(value);
    _ui->param4SB->blockSignals(value);
}

void WaypointsFeaturesWidget::clearFeatures()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _ui->identifierSB->setValue(0);
    _ui->sequenceSB->setValue(0);
    _ui->autocontCB->setCurrentIndex(0);
    _ui->taskUuidCB->setCurrentIndex(0);
    _ui->typeLE->clear();
    _ui->posXSB->setValue(0);
    _ui->posYSB->setValue(0);
    _ui->posZSB->setValue(0);
    _ui->rollSB->setValue(0);
    _ui->pitchSB->setValue(0);
    _ui->yawSB->setValue(0);
    _ui->param1SB->setValue(0);
    _ui->param2SB->setValue(0);
    _ui->param3SB->setValue(0);
    _ui->param4SB->setValue(0);

    _hasChanges = false;
    _ui->applyChangesB->setDisabled(true);
}

} // namespace gcs
