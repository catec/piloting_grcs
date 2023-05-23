#pragma once

#include <common/Types.h>

#include <QDialog>
#include <memory>
namespace Ui {
class DownloadDataDDHLDialog;
}

namespace gcs {
class CommsLinkDDHLDTO;

class DownloadDataDDHLDialog : public QDialog
{
    Q_OBJECT

  public:
    explicit DownloadDataDDHLDialog(QWidget* parent = nullptr);
    virtual ~DownloadDataDDHLDialog();

    QString getInspectionPlanUUID() const;
    void    setTargetInspectionPlanUUID(const QString&);

    void setDDHLCommsOptions(const CommsLinkDDHLDTO&);

    QString getInspectionPlanSite() const;

  Q_SIGNALS:
    void downloadInspectionPlanFromUUID();
    void downloadInspectionPlansFromSites(const SiteType&);

  private Q_SLOTS:
    void inspPlanEdited(const QString&);
    void changedDownloadMode(bool checked);

  private:
    void accept();
    void checkValidFields();

    std::unique_ptr<Ui::DownloadDataDDHLDialog> _ui;
};
} // namespace gcs
