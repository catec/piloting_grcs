
#include "WaypointsEditorWidget.h"

#include <QsLog/QsLog.h>

#include <QLayout>
#include <QSettings>
#include <QSizePolicy>

#include "dataModel/dtos/InspectionTaskDTO.h"
#include "dataModel/dtos/MissionDTO.h"
#include "ui_WaypointsEditorWidget.h"

namespace gcs {
WaypointsEditorWidget::WaypointsEditorWidget(QWidget* parent) :
        QWidget(parent), _ui(std::make_unique<Ui::WaypointsEditorWidget>())
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _ui->setupUi(this);
    loadGeometry();

    if (_isVertical) {
        setVerticalLayout();
    } else {
        setHorizontalLayout();
    }

    // clang-format off
   connect(&_waypointsTreeWidget,     &WaypointsTreeWidget::changeCurrentWaypoint,
           this,                      &WaypointsEditorWidget::changeCurrentWaypoint);
   connect(&_waypointsFeaturesWidget, &WaypointsFeaturesWidget::editWaypointFeatures,
           this,                      &WaypointsEditorWidget::editWaypointFeatures);
   connect(&_waypointsTreeWidget,     &WaypointsTreeWidget::editSelectedWaypoint,
           this,                      &WaypointsEditorWidget::editWaypointFeatures);
    // clang-format on

    updateStatus(false);
}

WaypointsEditorWidget::~WaypointsEditorWidget()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    writeGeometry();
}

void WaypointsEditorWidget::loadGeometry()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QSettings settings;
    settings.beginGroup("WaypointsEditorWidget");
    _isVertical = settings.value("isVertical", true).toBool();
    settings.endGroup();
}

void WaypointsEditorWidget::writeGeometry()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QSettings settings;
    settings.beginGroup("WaypointsEditorWidget");
    settings.setValue("isVertical", _isVertical);
    settings.endGroup();
}

void WaypointsEditorWidget::setVerticalLayout()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _isVertical = true;
    _waypointsTreeWidget.setSizePolicy(QSizePolicy(QSizePolicy::Preferred, QSizePolicy::MinimumExpanding));

    QVBoxLayout* layout = new QVBoxLayout();
    resetLayout(layout);
}

void WaypointsEditorWidget::setHorizontalLayout()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _isVertical = false;
    _waypointsTreeWidget.setSizePolicy(QSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Preferred));

    QHBoxLayout* layout = new QHBoxLayout();
    resetLayout(layout);
}

void WaypointsEditorWidget::resetLayout(QLayout* newLayout)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (newLayout) {
        if (layout()) {
            delete layout();
        }

        newLayout->addWidget(&_waypointsFeaturesWidget);
        newLayout->addWidget(&_waypointsTreeWidget);
        newLayout->setContentsMargins(1, 1, 1, 1);
        setLayout(newLayout);
    }
}

void WaypointsEditorWidget::updateMission(const MissionDTO& missionDTO)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _hasRoutes = !missionDTO.getLocalRoutesMap().isEmpty();

    _waypointsTreeWidget.redrawItems(missionDTO);
    _waypointsFeaturesWidget.updateFeatures(missionDTO);
}

void WaypointsEditorWidget::updateStatus(const bool isEnabled)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _waypointsFeaturesWidget.setEnableFeatures(isEnabled);
    _waypointsTreeWidget.setEnabled(isEnabled);
}

void WaypointsEditorWidget::updateInspectionTaskList(const QList<InspectionTaskDTO> inspectionTaskList)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _waypointsFeaturesWidget.updateInspectionTaskList(inspectionTaskList);
}
} // namespace gcs
