
#include "DownloadAssetFileDialog.h"

#include <QsLog/QsLog.h>
#include <dataModel/dtos/FileDTO.h>

#include <QMessageBox>

#include "ui_DownloadAssetFileDialog.h"

#define CHECK 0
#define NAME  1
#define VALUE 2

namespace gcs {

DownloadAssetFileDialog::DownloadAssetFileDialog(QWidget* parent) :
        QDialog(parent), _ui(std::make_unique<Ui::DownloadAssetFileDialog>())
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _ui->setupUi(this);
}

DownloadAssetFileDialog::~DownloadAssetFileDialog()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

void DownloadAssetFileDialog::setDownloadedFileList(const QList<FileDTO>& list)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _fileMapToDownload.clear();

    for (const auto& fileDTO : list) {
        insertMissionRoot(fileDTO);
    }

    _ui->treeWidget->resizeColumnToContents(CHECK);
    _ui->treeWidget->resizeColumnToContents(NAME);

    // _ui->treeWidget->setSelectionMode(QAbstractItemView::NoSelection);
}

FileDTO DownloadAssetFileDialog::getSelectedFile()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return _selectedFile;
}

void DownloadAssetFileDialog::insertMissionRoot(const FileDTO& file)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    auto treeItem = new QTreeWidgetItem(_ui->treeWidget);
    // treeItem->setFlags(Qt::NoItemFlags);
    treeItem->setText(NAME, file.getOriginalName());
    treeItem->setText(VALUE, "");
    treeItem->setExpanded(true);

    insertMissionItem(treeItem, "UUID", file.getUUID());
    insertMissionItem(treeItem, "CreationDateTime", file.getCreationDateTime().toString("yyyy-MM-dd hh:mm:ss"));
    insertMissionItem(treeItem, "LastUpdateDateTime", file.getLastUpdateDateTime().toString("yyyy-MM-dd hh:mm:ss"));
    insertMissionItem(treeItem, "URL", file.getUrl());
    insertMissionItem(treeItem, "Size (MB)", QString::number(file.getSize() * 10e-7));

    auto radioButton = new QRadioButton(this);
    radioButton->setStyleSheet(
            "QRadioButton { background-color: transparent;} QRadioButton::indicator:checked { background-color: "
            "white;} QRadioButton::indicator:unchecked { background-color: transparent;}");
    // myTreeWidget_ui->treeWidget->headerView()->resizeSection(CHECK, 30);
    _ui->treeWidget->setItemWidget(treeItem, CHECK, radioButton);
    _fileMapToDownload.insert(radioButton, file);
}

void DownloadAssetFileDialog::insertMissionItem(
        QTreeWidgetItem* parent, const QString& name, const QVariant& receivedValue)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    auto treeItem = new QTreeWidgetItem();
    if (receivedValue.canConvert<QString>()) {
        treeItem->setText(NAME, name);
        treeItem->setData(VALUE, Qt::DisplayRole, receivedValue);

        parent->addChild(treeItem);
    } else {
        QLOG_ERROR() << __PRETTY_FUNCTION__ << "Invalid input type";
    }
}

void DownloadAssetFileDialog::accept()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    bool any_selected{false};
    for (const auto& radioB : _fileMapToDownload.keys()) {
        if (radioB->isChecked()) {
            any_selected  = true;
            _selectedFile = _fileMapToDownload.value(radioB);
        }
    }

    if (!any_selected) {
        QMessageBox::warning(
                this,
                tr("Download Asset File from DDHL"),
                QString("You have not select any asset to download, please select one asset file to continue"));

        return;
    } else {
        const auto resp = QMessageBox::question(
                this,
                tr("Download Asset File from DDHL"),
                QString("Are you sure that you want to download the asset file %1 from DDHL?\n")
                        .arg(_selectedFile.getOriginalName()));
        if (resp == QMessageBox::No) {
            return;
        }
    }

    QDialog::accept();
}

} // namespace gcs
