
#include "CommsOptionsMavsdkDialog.h"

#include <QHostAddress>
#include <QMessageBox>

#include "ui_CommsOptionsMavsdkDialog.h"

namespace gcs {

CommsOptionsMavsdkDialog::CommsOptionsMavsdkDialog(QWidget* parent) :
        QDialog(parent), _ui(std::make_unique<Ui::CommsOptionsMavsdkDialog>())
{
    _ui->setupUi(this);
}

CommsOptionsMavsdkDialog::~CommsOptionsMavsdkDialog() {}

void CommsOptionsMavsdkDialog::setHeartbeatInterval(const quint16 value)
{
    _ui->heartbeatSB->setValue(value);
}

quint16 CommsOptionsMavsdkDialog::getHeartbeatInterval() const
{
    return _ui->heartbeatSB->value();
}

void CommsOptionsMavsdkDialog::setLocalSystemId(const quint16 value)
{
    _ui->localSystemIdSB->setValue(value);
}

quint16 CommsOptionsMavsdkDialog::getLocalSystemId() const
{
    return _ui->localSystemIdSB->value();
}

void CommsOptionsMavsdkDialog::setLocalComponentId(const quint16 value)
{
    _ui->localComponentIdSB->setValue(value);
}

quint16 CommsOptionsMavsdkDialog::getLocalComponentId() const
{
    return _ui->localComponentIdSB->value();
}

void CommsOptionsMavsdkDialog::setLocalIP(const QString text)
{
    _ui->localIpLE->setText(text);
}

QString CommsOptionsMavsdkDialog::getLocalIP() const
{
    return _ui->localIpLE->text();
}

void CommsOptionsMavsdkDialog::setLocalPort(const quint16 value)
{
    _ui->localPortSB->setValue(value);
}

quint16 CommsOptionsMavsdkDialog::getLocalPort() const
{
    return _ui->localPortSB->value();
}

void CommsOptionsMavsdkDialog::setTargetIP(const QString text)
{
    _ui->targetIpLE->setText(text);
}

QString CommsOptionsMavsdkDialog::getTargetIP() const
{
    return _ui->targetIpLE->text();
}

void CommsOptionsMavsdkDialog::setTargetPort(const quint16 value)
{
    _ui->targetPortSB->setValue(value);
}

quint16 CommsOptionsMavsdkDialog::getTargetPort() const
{
    return _ui->targetPortSB->value();
}

void CommsOptionsMavsdkDialog::setTargetSystemId(const quint16 value)
{
    _ui->targetSystemIdSB->setValue(value);
}

quint16 CommsOptionsMavsdkDialog::getTargetSystemId() const
{
    return _ui->targetSystemIdSB->value();
}

void CommsOptionsMavsdkDialog::accept()
{
    try {
        QHostAddress addr;
        if (!addr.setAddress(getLocalIP())) {
            throw std::runtime_error(
                    "The local IP address introduced has an invalid format. Please, introduce it again.");
        }

        if (!addr.setAddress(getTargetIP())) {
            throw std::runtime_error("Error in the target IP address");
        }

        /// \todo. We should check, in addition to the port being valid, that it is not in use.
        const quint16 upper_port = 34464;
        const quint16 lower_port = 1;
        if (getLocalPort() > upper_port || getLocalPort() < lower_port) {
            throw std::runtime_error("The local port introduced has an invalid format. Please, introduce it again.");
        }

        if (getTargetPort() > upper_port || getTargetPort() < lower_port) {
            throw std::runtime_error("Error in the target port");
        }

        QDialog::accept();
    } catch (std::runtime_error& e) {
        QMessageBox* msgBox = new QMessageBox(this);
        msgBox->setModal(true);
        msgBox->setAttribute(Qt::WA_DeleteOnClose);
        msgBox->setWindowTitle("Format error!");
        msgBox->setText(e.what());
        msgBox->setIcon(QMessageBox::Warning);
        msgBox->show();
    }
}

} // namespace gcs
