
#include "DownloadDataDDHLDialog.h"

#include <QsLog/QsLog.h>

#include <QIcon>
#include <QMessageBox>
#include <QStyle>

#include "dataModel/dtos/CommsLinkDDHLDTO.h"
#include "ui_DownloadDataDDHLDialog.h"

namespace gcs {

DownloadDataDDHLDialog::DownloadDataDDHLDialog(QWidget* parent) :
        QDialog(parent), _ui(std::make_unique<Ui::DownloadDataDDHLDialog>())
{
    _ui->setupUi(this);

    // clang-format off
    connect(_ui->inspPlanUUIDLE, &QLineEdit::textChanged,
            this,                &DownloadDataDDHLDialog::inspPlanEdited);
    
    connect(_ui->uuidRB,  &QRadioButton::toggled, 
            this,         &DownloadDataDDHLDialog::changedDownloadMode);
    // clang-format on
    _ui->uuidRB->setChecked(true);

    auto    icon   = style()->standardIcon(QStyle::SP_DialogHelpButton);
    QPixmap pixmap = icon.pixmap(_ui->iconL->size());

    _ui->iconL->setToolTip("Enter the UUID V4");
    _ui->iconL->setPixmap(pixmap);
}

DownloadDataDDHLDialog::~DownloadDataDDHLDialog() {}

void DownloadDataDDHLDialog::changedDownloadMode(bool checked)
{
    _ui->siteSelectionW->setEnabled(!checked);
    _ui->uuidSelectionW->setEnabled(checked);
}

QString DownloadDataDDHLDialog::getInspectionPlanUUID() const
{
    return _ui->inspPlanUUIDLE->text();
}

void DownloadDataDDHLDialog::setDDHLCommsOptions(const CommsLinkDDHLDTO& dto)
{
    auto endpointMode = dto.getEndpointMode();

    if (endpointMode == DDHLEndpointMode::URL) {
        _ui->ddhlUrlL->setText(dto.getTargetUrl());
        _ui->ddhlIPL->setText("-");
        _ui->ddhlPortL->setText("-");
    } else {
        _ui->ddhlUrlL->setText("-");
        _ui->ddhlIPL->setText(dto.getTargetIP());
        _ui->ddhlPortL->setText(QString::number(dto.getTargetPort()));
    }

    _ui->userL->setText(dto.getUsername());

    auto password = dto.getPassword();
    if (!password.isEmpty()) {
        _ui->passwordL->setText(password.fill('*'));
    }
}

void DownloadDataDDHLDialog::setTargetInspectionPlanUUID(const QString& uuid)
{
    _ui->inspPlanUUIDLE->setText(uuid);
}

void DownloadDataDDHLDialog::accept()
{
    try {
        checkValidFields();

        if (_ui->uuidRB->isChecked()) {
            if (_ui->inspPlanUUIDLE->text().isEmpty()) {
                throw std::runtime_error("Inspection plan UUID is empty");
            }

            /// \note. Check that uuid is valid
            const auto UUIDRegex = QRegExp("[0-9a-f]{8}-[0-9a-f]{4}-[1-5][0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}");
            if (!_ui->inspPlanUUIDLE->text().contains(UUIDRegex)) {
                throw std::runtime_error("Invalid UUID v4 format");
            }

            Q_EMIT downloadInspectionPlanFromUUID();
            QDialog::accept();
        } else if (_ui->sitesRB->isChecked()) {
            auto siteType = SiteType::UNKNOWN;
            if (_ui->viaductRB->isChecked()) {
                siteType = SiteType::VIADUCT;
            } else if (_ui->refineryRB->isChecked()) {
                siteType = SiteType::REFINERY;
            } else if (_ui->tunnelRB->isChecked()) {
                siteType = SiteType::TUNNEL;
            } else {
                throw std::runtime_error("Not avaliable site selected");
            }

            Q_EMIT downloadInspectionPlansFromSites(siteType);
            QDialog::accept();
        }

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

void DownloadDataDDHLDialog::inspPlanEdited(const QString& text)
{
    if (text.isEmpty()) {
        auto    icon   = style()->standardIcon(QStyle::SP_DialogHelpButton);
        QPixmap pixmap = icon.pixmap(_ui->iconL->size());

        _ui->iconL->setToolTip("Enter the UUID V4");
        _ui->iconL->setPixmap(pixmap);
        return;
    }

    const auto UUIDRegex = QRegExp("[0-9a-f]{8}-[0-9a-f]{4}-[1-5][0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}");
    QIcon      icon;
    if (!text.contains(UUIDRegex)) {
        icon = style()->standardIcon(QStyle::SP_MessageBoxWarning);
        _ui->iconL->setToolTip("UUID V4 format is invalid");
    } else {
        icon = style()->standardIcon(QStyle::SP_DialogOkButton);
        _ui->iconL->setToolTip("UUID V4 format is valid");
    }

    QPixmap pixmap = icon.pixmap(_ui->iconL->size());
    _ui->iconL->setPixmap(pixmap);
}

void DownloadDataDDHLDialog::checkValidFields()
{
    if (_ui->ddhlIPL->text().isEmpty()) {
        throw std::runtime_error("IP is empty");
    }

    if (_ui->ddhlPortL->text().isEmpty()) {
        throw std::runtime_error("Port is empty");
    }

    if (_ui->userL->text().isEmpty()) {
        throw std::runtime_error("Username is empty");
    }

    if (_ui->passwordL->text().isEmpty()) {
        throw std::runtime_error("Password is empty");
    }
}
} // namespace gcs
