#pragma once

#include <QDateTime>
#include <QDialog>
#include <memory>

namespace Ui {
class MissionInfoDialog;
}

namespace gcs {
class MissionInfoDialog : public QDialog
{
    Q_OBJECT

  public:
    explicit MissionInfoDialog(QWidget* parent = nullptr);
    virtual ~MissionInfoDialog();

    void setMissionName(const QString&) const;
    void setMissionDescription(const QString&) const;
    void setMissionCreationDate(const QString&) const;
    void setMissionLastUpdateDate(const QString&) const;

    const QString getMissionName();
    const QString getMissionDescription();

  private:
    std::unique_ptr<Ui::MissionInfoDialog> _ui;
};
} // namespace gcs
