#include "TelemetryTreeWidget.h"

#include <QsLog/QsLog.h>
#include <common/CommonDefines.h>
#include <dataModel/RobotAttitudeAngVelocityDTO.h>
#include <dataModel/RobotPositionVelocityNedDTO.h>

#include <QDateTime>

#include "../../dtos/GuiDTO.h"
#include "ui_TelemetryTreeWidget.h"

#define RAD2DEG 180. / M_PI

namespace gcs {

enum ItemID {
    CURR_INSP_ITEM,
    REACHED_INSP_ITEM,
    WP_TYPE,
    CURR_INSP_TASK,

    POS_X,
    POS_Y,
    POS_Z,
    SPEED_X,
    SPEED_Y,
    SPEED_Z,

    ATT_ROLL,
    ATT_PITCH,
    ATT_YAW,
    ATT_SPEED_ROLL,
    ATT_SPEED_PITCH,
    ATT_SPEED_YAW,

    BATT_VOLT,

    FLIGHT_STATUS,

    GPS_STATUS
};

TelemetryTreeWidget::TelemetryTreeWidget(GuiDTO& guiDTO, QWidget* parent) :
        QTreeWidget(parent), _ui(std::make_unique<Ui::TelemetryTreeWidget>()), _guiDTO(guiDTO), _updateTimer(this)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _ui->setupUi(this);

    insertItems();

    resetValues();

    expandAll();

    resizeColumnToContents(0);
    resizeColumnToContents(1);

    // clang-format off
    connect(&_updateTimer, &QTimer::timeout,
            this,          &TelemetryTreeWidget::updateWidget);
    _updateTimer.setInterval(1000); /// 1 Hz
    // clang-format on
}

TelemetryTreeWidget::~TelemetryTreeWidget()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

void TelemetryTreeWidget::setUpdateInterval(const int value)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (_updateTimer.interval() != value) {
        _updateTimer.setInterval(value);
        if (_updateTimer.isActive()) {
            _updateTimer.start();
        }
    }
}

void TelemetryTreeWidget::updateCommsStatus(const bool isConnected)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (isConnected) {
        _updateTimer.start();
    } else {
        _updateTimer.stop();
        resetValues();
    }
}

void TelemetryTreeWidget::update(QSharedPointer<RobotDataBaseDTO> idto)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (!idto) {
        return;
    }

    switch (idto->getDTOType()) {
        case DTOType::ROBOT_POSITION_VELOCITY_NED: {
            auto dto = idto.dynamicCast<RobotPositionVelocityNedDTO>();
            if (dto) {
                _lastRobotPositionVelocityDTO = dto;
            }
            break;
        }
        case DTOType::ROBOT_ATTITUDE_ANGVEL: {
            auto dto = idto.dynamicCast<RobotAttitudeAngVelocityDTO>();
            if (dto) {
                _lastRobotAttitudeAngVelocityDTO = dto;
            }
            break;
        }
        default: {
            break;
        }
    }
}

void TelemetryTreeWidget::update(QSharedPointer<CurrentInspItemDTO> dto)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _lastCurrentInspItemDTO = dto;
}

void TelemetryTreeWidget::update(QSharedPointer<ReachedInspItemDTO> dto)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _lastReachedInspItemDTO = dto;
}

void TelemetryTreeWidget::updateLogging(const QString& logFilePath)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (_loggingManager) {
        if (_loggingManager->getLogFilePath() == logFilePath) {
            return;
        }
        _loggingManager.reset();
    }

    if (logFilePath.isNull() || logFilePath.isEmpty()) {
        return;
    }

    const auto telemetryHeader = QString("## timestamp,task_uuid,pos_x,pos_y,pos_z,quat_x,quat_y,quat_z,quat_w");
    _loggingManager            = std::make_unique<LoggingManager>(10);
    if (_loggingManager) {
        _loggingManager->startLogging(logFilePath, telemetryHeader);
    }
}

void TelemetryTreeWidget::updateWidget()
{
    // QLOG_TRACE() << __PRETTY_FUNCTION__;

    updateItem(WP_TYPE, "-");
    updateItem(CURR_INSP_TASK, "-");

    auto lastCurrentInspItemDTO = _lastCurrentInspItemDTO;

    if (!_guiDTO.getLoadedRoute().getWpsList().isEmpty() && lastCurrentInspItemDTO
        && lastCurrentInspItemDTO->getIndex() != -1) {
        const auto wpList           = _guiDTO.getLoadedRoute().getWpsList();
        const auto current_wp_index = _lastCurrentInspItemDTO->getIndex();
        if (current_wp_index <= wpList.size() - 1) {
            updateItem(WP_TYPE, ToString(wpList.at(current_wp_index).getWpType()));
            updateItem(CURR_INSP_TASK, wpList.at(current_wp_index).getTaskUUID());
        }
    }

    if (lastCurrentInspItemDTO && lastCurrentInspItemDTO->getIndex() != -1) {
        updateItem(CURR_INSP_ITEM, QString::number(lastCurrentInspItemDTO->getIndex()));
    } else {
        updateItem(CURR_INSP_ITEM, "-");
    }

    auto lastReachedInspItemDTO = _lastReachedInspItemDTO;
    if (lastReachedInspItemDTO && lastReachedInspItemDTO->getIndex() != -1) {
        updateItem(REACHED_INSP_ITEM, QString::number(lastReachedInspItemDTO->getIndex()));
    } else {
        updateItem(REACHED_INSP_ITEM, "-");
    }

    auto lastVehiclePositionVelocityDTO = _lastRobotPositionVelocityDTO;
    if (lastVehiclePositionVelocityDTO) {
        update(*lastVehiclePositionVelocityDTO);
    }

    auto lastRobotAttitudeAngVelocityDTO = _lastRobotAttitudeAngVelocityDTO;
    if (lastRobotAttitudeAngVelocityDTO) {
        update(*lastRobotAttitudeAngVelocityDTO);
    }

    const QString currInspTaskUUID = _items[CURR_INSP_TASK]->data(1, Qt::DisplayRole).toString();
    if (_guiDTO.getCommsIsConnected() && lastVehiclePositionVelocityDTO && lastRobotAttitudeAngVelocityDTO
        && currInspTaskUUID != "-") {
        addTelemetryMsg(currInspTaskUUID, *lastVehiclePositionVelocityDTO, *lastRobotAttitudeAngVelocityDTO);
    }
}

void TelemetryTreeWidget::resetValues()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _lastRobotPositionVelocityDTO.reset(new RobotPositionVelocityNedDTO);
    _lastRobotAttitudeAngVelocityDTO.reset(new RobotAttitudeAngVelocityDTO);
    _lastCurrentInspItemDTO.reset(new CurrentInspItemDTO);
    _lastReachedInspItemDTO.reset(new ReachedInspItemDTO);

    updateWidget();
}

void TelemetryTreeWidget::insertItems()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    insertWaypointsIndexItems();
    insertRobotPositionVelocityItems();
    insertRobotAttitudeItems();
}

void TelemetryTreeWidget::insertWaypointsIndexItems()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    auto childItem = new QTreeWidgetItem(this, {"General Information"});
    insertItem(CURR_INSP_ITEM, childItem, "Current Inspection Item");
    insertItem(REACHED_INSP_ITEM, childItem, "Reached Inspection Item");
    insertItem(WP_TYPE, childItem, "Waypoint Type");
    insertItem(CURR_INSP_TASK, childItem, "Current Inspection Task");
}

void TelemetryTreeWidget::insertRobotPositionVelocityItems()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    auto childItemPos = new QTreeWidgetItem(this, {"Vehicle Position ENU"});
    insertItem(POS_X, childItemPos, "East (m)");
    insertItem(POS_Y, childItemPos, "North (m)");
    insertItem(POS_Z, childItemPos, "Up (m)");

    auto childItemVel = new QTreeWidgetItem(this, {"Vehicle Velocity ENU"});
    insertItem(SPEED_Y, childItemVel, "East (m/s)");
    insertItem(SPEED_X, childItemVel, "North (m/s)");
    insertItem(SPEED_Z, childItemVel, "Up (m/s)");
}

void TelemetryTreeWidget::insertRobotAttitudeItems()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    auto childItemOri = new QTreeWidgetItem(this, {"Vehicle Attitude"});
    insertItem(ATT_ROLL, childItemOri, "Roll (deg)");
    insertItem(ATT_PITCH, childItemOri, "Pitch (deg)");
    insertItem(ATT_YAW, childItemOri, "Yaw (deg)");

    auto childItemVel = new QTreeWidgetItem(this, {"Vehicle Angular Velocity"});
    insertItem(ATT_SPEED_ROLL, childItemVel, "Roll (rad/s)");
    insertItem(ATT_SPEED_PITCH, childItemVel, "Pitch (rad/s)");
    insertItem(ATT_SPEED_YAW, childItemVel, "Yaw (rad/s)");
}

void TelemetryTreeWidget::update(const RobotPositionVelocityNedDTO& dto)
{
    // QLOG_TRACE() << __PRETTY_FUNCTION__;

    updateItem(POS_X, QString::number(dto.getPositionX(), 'f', 5));
    updateItem(POS_Y, QString::number(dto.getPositionY(), 'f', 5));
    updateItem(POS_Z, QString::number(dto.getPositionZ(), 'f', 5));

    updateItem(SPEED_X, QString::number(dto.getNorthSpeed(), 'f', 5));
    updateItem(SPEED_Y, QString::number(dto.getEastSpeed(), 'f', 5));
    updateItem(SPEED_Z, QString::number(-1.0 * dto.getDownSpeed(), 'f', 5));
}

void TelemetryTreeWidget::update(const RobotAttitudeAngVelocityDTO& dto)
{
    // QLOG_TRACE() << __PRETTY_FUNCTION__;

    const auto eulerAngles
            = ToEulerAngles(dto.getQuaternionW(), dto.getQuaternionX(), dto.getQuaternionY(), dto.getQuaternionZ());

    updateItem(ATT_ROLL, QString::number(eulerAngles[0] * RAD2DEG, 'f', 5));
    updateItem(ATT_PITCH, QString::number(eulerAngles[1] * RAD2DEG, 'f', 5));
    updateItem(ATT_YAW, QString::number(eulerAngles[2] * RAD2DEG, 'f', 5));

    updateItem(ATT_SPEED_ROLL, QString::number(dto.getRollAngularVelocity(), 'f', 5));
    updateItem(ATT_SPEED_PITCH, QString::number(dto.getPitchAngularVelocity(), 'f', 5));
    updateItem(ATT_SPEED_YAW, QString::number(-1.0 * dto.getYawAngularVelocity(), 'f', 5));
}

void TelemetryTreeWidget::insertItem(
        const quint8 id, QTreeWidgetItem* parent, const QString& name, const QString& value)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _items.insert(id, new QTreeWidgetItem(parent, {name, value}));
}

void TelemetryTreeWidget::updateItem(const quint8 id, const QVariant& value)
{
    // QLOG_TRACE() << __PRETTY_FUNCTION__;

    _items[id]->setData(1, Qt::DisplayRole, value);
}

void TelemetryTreeWidget::addTelemetryMsg(
        const QString&                     inspTaskUUID,
        const RobotPositionVelocityNedDTO& posDTO,
        const RobotAttitudeAngVelocityDTO& attitudeDTO)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    const auto time_stamp = QDateTime::currentDateTime().toMSecsSinceEpoch();
    const auto msg        = QString("%1,%2,%3,%4,%5,%6,%7,%8,%9")
                             .arg(QString::number(time_stamp / 1000.0, 'f', 5))
                             .arg(inspTaskUUID)
                             .arg(QString::number(posDTO.getPositionX(), 'f', 3))
                             .arg(QString::number(posDTO.getPositionY(), 'f', 3))
                             .arg(QString::number(posDTO.getPositionZ(), 'f', 3))
                             .arg(QString::number(attitudeDTO.getQuaternionX(), 'f', 5))
                             .arg(QString::number(attitudeDTO.getQuaternionY(), 'f', 5))
                             .arg(QString::number(attitudeDTO.getQuaternionZ(), 'f', 5))
                             .arg(QString::number(attitudeDTO.getQuaternionW(), 'f', 5));

    if (_loggingManager) {
        _loggingManager->pullNewLogMessage(msg);
    }
}

} // namespace gcs
