#ifndef GUI_LIB_VIEW_POSE_DIALOG_H
#define GUI_LIB_VIEW_POSE_DIALOG_H

#include <QDialog>
#include <QLabel>
#include <QLineEdit>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QCloseEvent>

#include <functional>
#include <armadillo>
#include <thread>

#include <gui_lib/utils.h>

namespace as64_
{

namespace gui_
{

class ViewPoseDialog : public QDialog
{
  Q_OBJECT

public:
  ViewPoseDialog(std::function<arma::vec()> getPose, QWidget *parent = 0);
  ~ViewPoseDialog();

  void setTitle(const std::string &title_) { this->setWindowTitle(title_.c_str()); }

  void setUpdateRate(unsigned up_rate_ms) { up_rate_ms_ = up_rate_ms;  emit updateRateChangedSignal(); }

signals:
  void updateRateChangedSignal();

public slots:
  void launch();
  void stop();

private:
  MyLineEdit *x_le;
  MyLineEdit *y_le;
  MyLineEdit *z_le;
  MyLineEdit *qw_le;
  MyLineEdit *qx_le;
  MyLineEdit *qy_le;
  MyLineEdit *qz_le;

  unsigned up_rate_ms_;

  bool run;
  std::function<arma::vec()> get_pose_fun_;

  void updateDialogThread();

  MyLineEdit *createLineEdit();

  void closeEvent(QCloseEvent *event) override;
};

} // namespace gui_

} // namespace as64_

#endif // GUI_LIB_VIEW_POSE_DIALOG_H
