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
  ViewWrenchDialog(const std::map< std::string, std::function<arma::vec()> > &wrench_map_, QWidget *parent = 0);
  ViewWrenchDialog(const std::vector<std::function<arma::vec()>> &wrench_funs, const std::vector<std::string> &wrench_fun_names, QWidget *parent = 0);
  ViewWrenchDialog(std::function<arma::vec()> readWrench, std::function<arma::mat()> getRelRot, QWidget *parent = 0);
  ~ViewWrenchDialog();

  void setTitle(const std::string &title_) { this->setWindowTitle(title_.c_str()); }

  void setUpdateRate(unsigned up_rate_ms) { up_rate_ms_ = up_rate_ms;  emit updateRateChangedSignal(); }

signals:
  void updateRateChangedSignal();

public slots:
  void launch();
  void stop();

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

  std::function<arma::vec()> get_wrench;

  void updateDialogThread();

  MyLineEdit *createLineEdit();

  void closeEvent(QCloseEvent *event) override;

  std::map< std::string, std::function<arma::vec()> > wrench_map;

  void init(const std::map< std::string, std::function<arma::vec()> > &wrench_map_);
};

} // namespace gui_

} // namespace as64_

#endif // GUI_LIB_VIEW_WRENCH_DIALOG_H
