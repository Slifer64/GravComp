#ifndef SET_POSES_DIALOG_H
#define SET_POSES_DIALOG_H

#include <QLineEdit>
#include <QComboBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QComboBox>
#include <QCheckBox>
#include <QScrollArea>
#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QDialog>
#include <QSpacerItem>
#include <QPalette>

#include <armadillo>

class PoseWidget;
class AddPoseDialog;

// =======================================================
// ===============    SetPosesDialog    ==================
// =======================================================

class SetPosesDialog : public QDialog
{
Q_OBJECT

public:
  explicit SetPosesDialog(int N_joints, QWidget *parent = 0);
  ~SetPosesDialog();

  std::vector<arma::vec> poses;

public slots:
  void launch();
  void stop();

private slots:
  void onOkBtnClicked();
  void onRemoveBtnClicked();
  void onAddBtnClicked();
  void onEditBtnClicked();
  void addNewPose();

private:
  QWidget *central_widget;

  int N_JOINTS;

  std::list<PoseWidget *> poses_widget;
  QVBoxLayout *poses_layout;

  QScrollArea *scroll_area;
  QWidget *scroll_area_contents;

  QPushButton *ok_btn;
  QPushButton *cancel_btn;
  QPushButton *add_pose_btn;
  QPushButton *edit_pose_btn;
  QPushButton *remove_pose_btn;

  AddPoseDialog *add_pose_dialog;
};

// ===================================================
// ===============    PoseWidget    ==================
// ===================================================

class PoseWidget: public QWidget
{
    Q_OBJECT

public:
    PoseWidget(int N, const QString &label, QWidget *parent=0);
    ~PoseWidget();

    bool isChecked() const;
    void setChecked(bool set);
    void setReadOnly(bool set);
    void setPose(const arma::vec &pose);
    void setLabel(const QString &label);
    arma::vec getPose() const;

private:
    std::vector<QLineEdit *> jpos_le;
    QCheckBox *chk_box;
    QHBoxLayout *layout;
};

// ======================================================
// ===============    AddPoseDialog    ==================
// ======================================================

class AddPoseDialog: public QDialog
{
    Q_OBJECT

public:
    AddPoseDialog(int N, const QString &label, QWidget *parent=0);
    ~AddPoseDialog();
    arma::vec getPose() const;

signals:
    void newPoseEntered();

private:
    std::vector<QLineEdit *> jpos_le;
    QLabel *pose_label;
    QPushButton *ok_btn;
    QPushButton *cancel_btn;
};

#endif // SET_POSES_DIALOG_H
