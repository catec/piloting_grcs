
#include "RobotAlarmsWidget.h"

#include <QsLog/QsLog.h>

#include <QTreeWidget>

#include "communications/MavSDK/dtos/AlarmListDTO.h"
#include "communications/MavSDK/dtos/AlarmStatusDTO.h"
#include "ui_RobotAlarmsWidget.h"

#define STATUS 0
#define ID     1
#define NAME   2
#define ERRORS 3
#define WARNS  4

namespace gcs {

RobotAlarmsWidget::RobotAlarmsWidget(QWidget* parent) : QWidget(parent), _ui(std::make_unique<Ui::RobotAlarmsWidget>())
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _ui->setupUi(this);
    updateAlarm(_ui->commsIndicatorL, AlarmStatusType::ERROR);

    for (int idx = 0; idx < _ui->treeWidget->columnCount(); idx++) {
        _ui->treeWidget->resizeColumnToContents(idx);
    }
}

RobotAlarmsWidget::~RobotAlarmsWidget()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

void RobotAlarmsWidget::updateCommsStatus(const bool isConnected)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (isConnected) {
        updateAlarm(_ui->commsIndicatorL, AlarmStatusType::OK);
    } else {
        updateAlarm(_ui->commsIndicatorL, AlarmStatusType::ERROR);
    }
}

void RobotAlarmsWidget::setAlarmList(const AlarmListDTO& alarmListDTO)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    blockSignals(true);

    _ui->treeWidget->clear();

    for (const auto& alarm : alarmListDTO.getAlarmList()) {
        QTreeWidgetItem* taskItem = new QTreeWidgetItem;
        if (!taskItem) {
            continue;
        }

        taskItem->setFlags(Qt::NoItemFlags);
        // taskItem->setFlags(static_cast<Qt::ItemFlags>(
        //                         Qt::ItemIsSelectable |
        //                         Qt::ItemIsEnabled));

        taskItem->setData(ID, Qt::DisplayRole, alarm.getIndex());
        taskItem->setData(NAME, Qt::DisplayRole, alarm.getName());
        taskItem->setToolTip(NAME, alarm.getDescription());

        _ui->treeWidget->addTopLevelItem(taskItem);

        QLabel* label = new QLabel(this);
        updateAlarm(label, AlarmStatusType::UNKNOWN);
        _ui->treeWidget->setItemWidget(taskItem, STATUS, label);

        _items.insert(alarm.getIndex(), taskItem);

        /// \note. If not receive update of alarm status in 5 seconds, set it to unknown
        QTimer* currItemTimer = new QTimer(this);
        connect(currItemTimer, &QTimer::timeout, this, &RobotAlarmsWidget::updateLabelTimedOut);
        currItemTimer->setSingleShot(true);
        currItemTimer->setInterval(5000); /// 5 Hz

        _itemsTimers.insert(alarm.getIndex(), currItemTimer);
    }

    blockSignals(false);
    _ui->treeWidget->sortItems(ID, Qt::AscendingOrder);
    for (int idx = 0; idx < _ui->treeWidget->columnCount(); idx++) {
        _ui->treeWidget->resizeColumnToContents(idx);
    }
}

void RobotAlarmsWidget::resetAlarmList()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    blockSignals(true);

    _ui->treeWidget->clear();
    _items.clear();
    _itemsTimers.clear();

    blockSignals(false);
}

void RobotAlarmsWidget::updateLabelTimedOut()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    const auto timer = qobject_cast<QTimer*>(sender());
    if (!timer) {
        return;
    }

    const auto index = _itemsTimers.key(timer);
    if (index == -1) {
        return;
    }

    const auto current_item = _items[index];
    if (!current_item) {
        return;
    }

    QLOG_WARN() << "Timed out alarm with index: " << index;

    const auto label = qobject_cast<QLabel*>(_ui->treeWidget->itemWidget(current_item, STATUS));
    updateAlarm(label, AlarmStatusType::UNKNOWN);
}

void RobotAlarmsWidget::update(const AlarmStatusDTO& alarmStatusDTO)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (_items.empty()) {
        return;
    }

    const auto current_item = _items[alarmStatusDTO.getIndex()];
    if (!current_item) {
        QLOG_ERROR() << __PRETTY_FUNCTION__ << "Invalid alarm index to update: " << alarmStatusDTO.getIndex();
        return;
    }

    current_item->setData(ERRORS, Qt::DisplayRole, QString::number(alarmStatusDTO.getErrorsCount()));
    current_item->setData(WARNS, Qt::DisplayRole, QString::number(alarmStatusDTO.getWarningsCount()));

    const auto label = qobject_cast<QLabel*>(_ui->treeWidget->itemWidget(current_item, STATUS));
    updateAlarm(label, alarmStatusDTO.getAlarmStatus());

    _itemsTimers[alarmStatusDTO.getIndex()]->start();
}

void RobotAlarmsWidget::updateAlarm(QLabel* label, const AlarmStatusType& status)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (!label) {
        return;
    }

    switch (status) {
        case AlarmStatusType::OK: {
            label->setStyleSheet("QLabel { background-color: green; color: black; }");
            break;
        }
        case AlarmStatusType::ERROR: {
            label->setStyleSheet("QLabel { background-color: red; color: black;  }");
            break;
        }
        case AlarmStatusType::WARNING: {
            label->setStyleSheet("QLabel { background-color: yellow; color: black; }");
            break;
        }
        case AlarmStatusType::UNKNOWN: {
            label->setStyleSheet("QLabel { background-color: grey; color: white; }");
            break;
        }
        default: {
            QLOG_WARN() << "RobotAlarmsWidget::updateAlarm() -"
                        << QString("Unknown alarm status type: %1").arg(static_cast<uint8_t>(status));

            break;
        }
    }
}

} // namespace gcs
