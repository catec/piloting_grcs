#pragma once

#include <QDialog>
#include <QMap>
#include <QTreeWidgetItem>
#include <memory>

namespace Ui {
class SelectInspecionPlanFromSiteDialog;
}

namespace gcs {
class SiteListDTO;

class SelectInspecionPlanFromSiteDialog : public QDialog
{
    Q_OBJECT

  public:
    explicit SelectInspecionPlanFromSiteDialog(QWidget* parent = nullptr);
    virtual ~SelectInspecionPlanFromSiteDialog();

    void setSiteList(const SiteListDTO&);

  Q_SIGNALS:
    void selectedInspectionPlan(const QString&);

  private:
    void               accept();
    QList<SiteListDTO> obtainListFromEachSite(const SiteListDTO&);

    std::unique_ptr<Ui::SelectInspecionPlanFromSiteDialog> _ui;

    QMap<QString, QTreeWidgetItem*> _items;
};

} // namespace gcs
