#pragma once

#include <QCheckBox>
#include <QDialog>
#include <QMap>
#include <QPushButton>
#include <QTreeWidgetItem>
#include <memory>

namespace Ui {
class UploadDDHLManagerDialog;
}

namespace gcs {

class MissionResultDTO;

class UploadDDHLManagerDialog : public QDialog
{
    Q_OBJECT

  public:
    explicit UploadDDHLManagerDialog(QWidget* parent = nullptr);
    virtual ~UploadDDHLManagerDialog();

    void setMissionResultList(const QList<MissionResultDTO>&);

    QStringList getMissionsToUpload();

  private:
    void accept();

    QPushButton* createCircularButton(const int, std::function<void()>);

    void insertMissionRoot(const MissionResultDTO&);
    void insertMissionItem(QTreeWidgetItem*, const QString&, const QVariant&);

    std::unique_ptr<Ui::UploadDDHLManagerDialog> _ui;

    QMap<QCheckBox*, MissionResultDTO> _missionMapToUpload;
    QStringList                        _missionsToUpload;
};

} // namespace gcs
