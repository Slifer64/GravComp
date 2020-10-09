#ifndef GUI_LIB_VIEW_WRENCH_DIALOG_H
#define GUI_LIB_VIEW_WRENCH_DIALOG_H

#include <QDialog>
#include <QLabel>
#include <QLineEdit>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QCloseEvent>
#include <QComboBox>

#include <functional>
#include <armadillo>
#include <thread>

#include <gui_lib/utils.h>

namespace as64_
{

namespace gui_
{

class ViewWrenchDialog : public QDialog
{
  Q_OBJECT

public:
  ViewWrenchDialog(std::function<arma::vec()> readWrench, std::function<arma::mat()> getRelRot, QWidget *parent = 0);
  ~ViewWrenchDialog();

  void setTitle(const std::string &title_) { this->setWindowTitle(title_.c_str()); }

  void setUpdateRate(unsigned up_rate_ms) { up_rate_ms_ = up_rate_ms; }

public slots:
  void launch();
  void stop();

private slots:
  void refFrameChangedSlot(const QString &ref_frame);

private:
  MyLineEdit *fx_le;
  MyLineEdit *fy_le;
  MyLineEdit *fz_le;
  MyLineEdit *tx_le;
  MyLineEdit *ty_le;
  MyLineEdit *tz_le;

  QComboBox *ref_frame_cmbx;

  unsigned up_rate_ms_;

  bool run;
  std::function<arma::vec()> read_wrench;
  std::function<arma::mat()> get_rel_rot;

  std::function<arma::vec()> get_wrench;
  arma::vec getLocalWrench();
  arma::vec getBaseWrench();

  void updateDialogThread();

  MyLineEdit *createLineEdit();

  void closeEvent(QCloseEvent *event) override;
};

} // namespace gui_

} // namespace as64_

#endif // GUI_LIB_VIEW_WRENCH_DIALOG_H
