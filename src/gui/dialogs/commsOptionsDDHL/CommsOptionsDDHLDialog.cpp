
#include "CommsOptionsDDHLDialog.h"

#include <QsLog/QsLog.h>

#include <QAction>
#include <QHostAddress>
#include <QMessageBox>

#include "ui_CommsOptionsDDHLDialog.h"

namespace gcs {

CommsOptionsDDHLDialog::CommsOptionsDDHLDialog(QWidget* parent) :
        QDialog(parent), _ui(std::make_unique<Ui::CommsOptionsDDHLDialog>())
{
    _ui->setupUi(this);

    QAction* action = _ui->passwordLE->addAction(QIcon(":/images/eye-off"), QLineEdit::TrailingPosition);
    _passwordB      = qobject_cast<QToolButton*>(action->associatedWidgets().last());
    _passwordB->setCursor(QCursor(Qt::PointingHandCursor));

    // clang-format off
    connect(_ui->ipRB,  &QRadioButton::toggled, 
            this,       &CommsOptionsDDHLDialog::endpointTypeChanged);
    connect(_passwordB, &QToolButton::pressed, [&]()
    {
        _passwordB->setIcon(QIcon(":/images/eye-on"));
        _ui->passwordLE->setEchoMode(QLineEdit::Normal);
    });
    connect(_passwordB, &QToolButton::released, [&]()
    {
        _passwordB->setIcon(QIcon(":/images/eye-off"));
        _ui->passwordLE->setEchoMode(QLineEdit::Password);
    });
    // clang-format on
}

CommsOptionsDDHLDialog::~CommsOptionsDDHLDialog() {}

void CommsOptionsDDHLDialog::endpointTypeChanged(bool ipChecked)
{
    _ui->urlLE->setEnabled(!ipChecked);
    _ui->ipLE->setEnabled(ipChecked);
    _ui->portSB->setEnabled(ipChecked);
}

void CommsOptionsDDHLDialog::setEndpointMode(const DDHLEndpointMode& type)
{
    const auto ipState = type == DDHLEndpointMode::IP;
    _ui->ipRB->setChecked(ipState);
    _ui->urlRB->setChecked(!ipState);

    Q_EMIT endpointTypeChanged(ipState);
}

DDHLEndpointMode CommsOptionsDDHLDialog::getEndpointMode() const
{
    if (_ui->urlRB->isChecked()) {
        return DDHLEndpointMode::URL;
    }
    return DDHLEndpointMode::IP;
}

void CommsOptionsDDHLDialog::setTargetUrl(const QString& text)
{
    _ui->urlLE->setText(text);
}

QString CommsOptionsDDHLDialog::getTargetUrl() const
{
    return _ui->urlLE->text();
}

void CommsOptionsDDHLDialog::setTargetIP(const QString& text)
{
    _ui->ipLE->setText(text);
}

QString CommsOptionsDDHLDialog::getTargetIP() const
{
    return _ui->ipLE->text();
}

void CommsOptionsDDHLDialog::setTargetPort(const quint16& value)
{
    _ui->portSB->setValue(value);
}

quint16 CommsOptionsDDHLDialog::getTargetPort() const
{
    return _ui->portSB->value();
}

void CommsOptionsDDHLDialog::setUsername(const QString& text)
{
    _ui->userLE->setText(text);
}

QString CommsOptionsDDHLDialog::getUsername() const
{
    return _ui->userLE->text();
}

void CommsOptionsDDHLDialog::setPassword(const QString& text)
{
    _ui->passwordLE->setText(text);
}

QString CommsOptionsDDHLDialog::getPassword() const
{
    return _ui->passwordLE->text();
}

void CommsOptionsDDHLDialog::accept()
{
    try {
        if (_ui->urlRB->isChecked()) {
            if (_ui->urlLE->text().isEmpty()) {
                throw std::runtime_error("Url is empty");
            }
        } else {
            if (_ui->passwordLE->text().isEmpty()) {
                throw std::runtime_error("Password is empty");
            }

            if (_ui->userLE->text().isEmpty()) {
                throw std::runtime_error("User is empty");
            }

            QHostAddress addr;
            if (!addr.setAddress(getTargetIP())) {
                throw std::runtime_error("Error in the target IP address");
            }

            /// \todo. We should check, in addition to the port being valid, that it is not in use.
            const quint16 upper_port = 34464;
            const quint16 lower_port = 1;
            if (getTargetPort() > upper_port || getTargetPort() < lower_port) {
                throw std::runtime_error("Error in the target port");
            }
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
