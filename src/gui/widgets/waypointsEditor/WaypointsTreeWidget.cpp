
#include "WaypointsTreeWidget.h"

#include <QsLog/QsLog.h>
#include <common/CommonDefines.h>

#include <iostream>

#include "dataModel/dtos/MissionDTO.h"
#include "ui_WaypointsTreeWidget.h"

#define ROUTE_ID  0
#define AUTOCONT  1
#define WP_SEQ    2
#define WP_ID     3
#define TASK_UUID 4
#define POSX      5
#define POSY      6
#define POSZ      7
#define ROLL      8
#define PITCH     9
#define YAW       10
#define PARAM1    11
#define PARAM2    12
#define PARAM3    13
#define PARAM4    14
#define TYPE      15

namespace gcs {

WaypointsTreeWidget::WaypointsTreeWidget(QWidget* parent) :
        QTreeWidget(parent), _ui(std::make_unique<Ui::WaypointsTreeWidget>())
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _ui->setupUi(this);

    // clang-format off
    connect(this,  &QTreeWidget::itemSelectionChanged,
            this,  &WaypointsTreeWidget::waypointSelectionChanged);
    // clang-format on

    for (int idx = 0; idx < this->columnCount(); idx++) {
        this->resizeColumnToContents(idx);
    }

    this->setColumnHidden(WP_ID, true);
}

WaypointsTreeWidget::~WaypointsTreeWidget()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    clear();
}

void WaypointsTreeWidget::redrawItems(const MissionDTO& missionDTO)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    blockSignals(true);

    clear();

    QTreeWidgetItem* selectedItem = nullptr;

    for (const auto& route : missionDTO.getLocalRoutesMap().values()) {
        QTreeWidgetItem* routeItem = new QTreeWidgetItem;
        if (routeItem) {
            routeItem->setFlags(Qt::ItemIsEnabled);
            routeItem->setData(ROUTE_ID, Qt::DisplayRole, route.getId());

            addTopLevelItem(routeItem);

            /// \note Sequence number starts with 0 because the route doesn't contain the home position.
            quint16 seqNumber = 0;
            for (const auto& waypoint : route.getWpsList()) {
                QTreeWidgetItem* waypointItem = new QTreeWidgetItem;
                if (waypointItem) {
                    waypointItem->setFlags(static_cast<Qt::ItemFlags>(Qt::ItemIsSelectable | Qt::ItemIsEnabled));

                    QVariant   wpSeqNumber(seqNumber++);
                    QVariant   wpId(waypoint.getId());
                    QVariant   wpTaskUuid(waypoint.getTaskUUID());
                    QVariant   posX(QString::number(waypoint.getPosition().getX(), 'f', 3));
                    QVariant   posY(QString::number(waypoint.getPosition().getY(), 'f', 3));
                    QVariant   posZ(QString::number(waypoint.getPosition().getZ(), 'f', 3));
                    const auto eaDeg = ToEulerAnglesDeg(
                            waypoint.getOrientation().getW(),
                            waypoint.getOrientation().getX(),
                            waypoint.getOrientation().getY(),
                            waypoint.getOrientation().getZ());
                    QVariant rollAngleDeg(QString::number(eaDeg[0], 'f', 3));
                    QVariant pitchAngleDeg(QString::number(eaDeg[1], 'f', 3));
                    QVariant yawAngleDeg(QString::number(eaDeg[2], 'f', 3));
                    QVariant param1(QString::number(waypoint.getActionParameters().getParam1(), 'f', 3));
                    QVariant param2(QString::number(waypoint.getActionParameters().getParam2(), 'f', 3));
                    QVariant param3(QString::number(waypoint.getActionParameters().getParam3(), 'f', 3));
                    QVariant param4(QString::number(waypoint.getActionParameters().getParam4(), 'f', 3));

                    auto     wpType = waypoint.getWpType();
                    QVariant type(QString(ToString(waypoint.getWpType())));

                    waypointItem->setCheckState(AUTOCONT, waypoint.getAutoContinue() ? Qt::Checked : Qt::Unchecked);

                    waypointItem->setData(WP_SEQ, Qt::DisplayRole, wpSeqNumber);
                    waypointItem->setData(WP_ID, Qt::DisplayRole, wpId);
                    waypointItem->setData(TASK_UUID, Qt::DisplayRole, wpTaskUuid);
                    waypointItem->setData(POSX, Qt::DisplayRole, posX);
                    waypointItem->setData(POSY, Qt::DisplayRole, posY);
                    waypointItem->setData(POSZ, Qt::DisplayRole, posZ);
                    waypointItem->setData(TYPE, Qt::DisplayRole, type);
                    if (wpType == WaypointType::POSE) {
                        waypointItem->setData(ROLL, Qt::DisplayRole, rollAngleDeg);
                        waypointItem->setData(PITCH, Qt::DisplayRole, pitchAngleDeg);
                        waypointItem->setData(YAW, Qt::DisplayRole, yawAngleDeg);
                        waypointItem->setData(PARAM1, Qt::DisplayRole, "----");
                        waypointItem->setData(PARAM2, Qt::DisplayRole, "----");
                        waypointItem->setData(PARAM3, Qt::DisplayRole, "----");
                        waypointItem->setData(PARAM4, Qt::DisplayRole, "----");
                    } else if (wpType == WaypointType::ACTION) {
                        waypointItem->setData(ROLL, Qt::DisplayRole, "----");
                        waypointItem->setData(PITCH, Qt::DisplayRole, "----");
                        waypointItem->setData(YAW, Qt::DisplayRole, "----");
                        waypointItem->setData(PARAM1, Qt::DisplayRole, param1);
                        waypointItem->setData(PARAM2, Qt::DisplayRole, param2);
                        waypointItem->setData(PARAM3, Qt::DisplayRole, param3);
                        waypointItem->setData(PARAM4, Qt::DisplayRole, param4);
                    }
                    routeItem->addChild(waypointItem);

                    if (waypoint.getId() == missionDTO.getCurrentWaypointId()) {
                        selectedItem = waypointItem;
                    }
                }
            }
        }
    }

    if (selectedItem) {
        setCurrentItem(selectedItem);
    }

    blockSignals(false);
}

void WaypointsTreeWidget::waypointSelectionChanged()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QList<QTreeWidgetItem*> selectItems = selectedItems();
    if (!selectItems.isEmpty()) {
        QTreeWidgetItem* selectItem = selectItems.at(0);
        if (selectItem) {
            auto dtoWaypoint     = createSharedDTO<WaypointDTO>();
            dtoWaypoint->getId() = selectItem->data(WP_ID, Qt::DisplayRole).toUInt();

            Q_EMIT changeCurrentWaypoint(dtoWaypoint);
        }
    }
}

} // namespace gcs
