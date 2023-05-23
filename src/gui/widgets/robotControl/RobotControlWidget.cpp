
#include "RobotControlWidget.h"

#include <QsLog/QsLog.h>
#include <dataModel/RouteDTO.h>

#include <QPushButton>

#include "ui_RobotControlWidget.h"

namespace gcs {

RobotControlWidget::RobotControlWidget(QWidget* parent) :
        QWidget(parent), _ui(std::make_unique<Ui::RobotControlWidget>())
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _ui->setupUi(this);

    // clang-format off
    connect(_ui->uploadWaypointListB,       &QPushButton::clicked,
            this,                           &RobotControlWidget::actionButtonClicked);
    connect(_ui->downloadWaypointListB,     &QPushButton::clicked,
            this,                           &RobotControlWidget::actionButtonClicked);
    connect(_ui->downloadCheckListB,        &QPushButton::clicked,
            this,                           &RobotControlWidget::actionButtonClicked);
    connect(_ui->setCurrWaypointItemB,      &QPushButton::clicked,
            this,                           &RobotControlWidget::actionButtonClicked);
    connect(_ui->downloadAlarmslistB,       &QPushButton::clicked,
            this,                           &RobotControlWidget::actionButtonClicked);
    connect(_ui->downloadHighLevelActionsB, &QPushButton::clicked,
            this,                           &RobotControlWidget::actionButtonClicked);
    connect(_ui->sendHomeB,                 &QPushButton::clicked,
            this,                           &RobotControlWidget::actionButtonClicked);
    // clang-format on
}

RobotControlWidget::~RobotControlWidget()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

void RobotControlWidget::updateCommsStatus(const bool isConnected)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QList<QPushButton*> actionButtons = _ui->generalW->findChildren<QPushButton*>();
    for (const auto& btn : actionButtons) {
        btn->setEnabled(isConnected);
    }

    _ui->waypointItemIdSB->setEnabled(isConnected);
}

void RobotControlWidget::updateLoadedRoute(const RouteDTO& loadedRoute)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    const bool hasLoadedRoute = loadedRoute.getWpsList().isEmpty() ? false : true;

    _ui->waypointItemIdSB->setEnabled(hasLoadedRoute);
    _ui->setCurrWaypointItemB->setEnabled(hasLoadedRoute);

    if (hasLoadedRoute) {
        auto maxSeqNumber = loadedRoute.getWpsList().size() - 1;
        _ui->waypointItemIdSB->setMaximum(maxSeqNumber);
        _ui->setCurrWaypointItemB->setToolTip("");
    }
}

void RobotControlWidget::actionButtonClicked()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    auto sender = QObject::sender();
    if (sender == _ui->uploadWaypointListB) {
        Q_EMIT uploadWaypointList();
    } else if (sender == _ui->downloadWaypointListB) {
        Q_EMIT downloadWaypointList();
    } else if (sender == _ui->downloadCheckListB) {
        Q_EMIT downloadCheckList();
    } else if (sender == _ui->setCurrWaypointItemB) {
        Q_EMIT setCurrentWaypointItem(_ui->waypointItemIdSB->value());
    } else if (sender == _ui->downloadAlarmslistB) {
        Q_EMIT downloadAlarmsList();
    } else if (sender == _ui->downloadHighLevelActionsB) {
        Q_EMIT downloadHighLevelActions();
    } else if (sender == _ui->sendHomeB) {
        Q_EMIT sendHome();
    } else {
        QLOG_WARN() << __PRETTY_FUNCTION__ << " - Unknown action button clicked";
    }
}

} // namespace gcs
