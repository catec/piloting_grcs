#include "SelectInspecionPlanFromSiteDialog.h"

#include <QsLog/QsLog.h>
#include <dataModel/dtos/SiteListDTO.h>

#include <QMessageBox>
#include <QTreeWidgetItem>

#include "ui_SelectInspecionPlanFromSiteDialog.h"

#define SITE            0
#define ASSET_SUBTYPE   1
#define INSP_PLAN_NAME  2
#define UPDATE_DATETIME 3
#define ASSET_NAME      4

namespace gcs {

SelectInspecionPlanFromSiteDialog::SelectInspecionPlanFromSiteDialog(QWidget* parent) :
        QDialog(parent), _ui(std::make_unique<Ui::SelectInspecionPlanFromSiteDialog>())
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _ui->setupUi(this);

    _ui->treeWidget->resizeColumnToContents(SITE);
    _ui->treeWidget->resizeColumnToContents(INSP_PLAN_NAME);
    _ui->treeWidget->resizeColumnToContents(UPDATE_DATETIME);
    _ui->treeWidget->resizeColumnToContents(ASSET_NAME);
    _ui->treeWidget->resizeColumnToContents(ASSET_SUBTYPE);

    _ui->treeWidget->setSortingEnabled(true);
}

SelectInspecionPlanFromSiteDialog::~SelectInspecionPlanFromSiteDialog()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

void SelectInspecionPlanFromSiteDialog::setSiteList(const SiteListDTO& siteList)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    blockSignals(true);

    _ui->treeWidget->clear();

    for (const auto& site : siteList.getSiteList()) {
        QList<QTreeWidgetItem*> items = _ui->treeWidget->findItems(site.getName(), SITE);
        QTreeWidgetItem*        parentItem;
        if (items.size() == 0) {
            parentItem = new QTreeWidgetItem(_ui->treeWidget);
            parentItem->setData(SITE, Qt::DisplayRole, site.getName());
        } else {
            parentItem = items.first();
        }
        QTreeWidgetItem* childItem = new QTreeWidgetItem(parentItem);
        childItem->setData(ASSET_SUBTYPE, Qt::DisplayRole, site.getSubtype());
        childItem->setData(INSP_PLAN_NAME, Qt::DisplayRole, site.getBasicInspectionPlan().getName());
        childItem->setData(UPDATE_DATETIME, Qt::DisplayRole, site.getBasicInspectionPlan().getLastUpdateDateTime());
        childItem->setData(ASSET_NAME, Qt::DisplayRole, site.getBasicAsset().getName());
        parentItem->addChild(childItem);
        _items.insert(site.getBasicInspectionPlan().getUUID(), childItem);
    }
    _ui->treeWidget->resizeColumnToContents(SITE);

    blockSignals(false);
}

void SelectInspecionPlanFromSiteDialog::accept()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    auto selectedItems = _ui->treeWidget->selectedItems();

    if (selectedItems.size() == 0) {
        QMessageBox::warning(
                this,
                tr("Inspection Plan Selection"),
                QString("Please, select any of the inspection plans before continue."));
        return;
    }

    auto selectedItem = selectedItems.first();
    if (selectedItem->data(SITE, Qt::DisplayRole).toString() != "") {
        QMessageBox::warning(
                this,
                tr("Inspection Plan Selection"),
                QString("Please, select any of the inspection plans before continue."));
        return;
    }

    const auto inspectionPlanUUID = _items.key(selectedItem);
    const auto inspectionPlanName = selectedItem->data(INSP_PLAN_NAME, Qt::DisplayRole).toString();
    const auto assetName          = selectedItem->data(ASSET_NAME, Qt::DisplayRole).toString();

    const auto response = QMessageBox::question(
            this,
            tr("Inspection Plan Selection"),
            QString("Do you want to download the inspection plan with name: '%1' and asset name: '%2' ?")
                    .arg(inspectionPlanName)
                    .arg(assetName));

    if (response == QMessageBox::Yes) {
        Q_EMIT selectedInspectionPlan(inspectionPlanUUID);
        QDialog::accept();
    }
}
} // namespace gcs
