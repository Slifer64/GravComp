#ifndef GUI_LIB_VIEW_JOINT_POSITION_DIALOG_H
#define GUI_LIB_VIEW_JOINT_POSITION_DIALOG_H

#include <QDialog>
#include <QLabel>
#include <QLineEdit>
#include <QSlider>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QCloseEvent>
#include <QComboBox>

#include <vector>
#include <cstring>
#include <armadillo>
#include <functional>
#include <thread>
#include <chrono>

namespace as64_
{

namespace gui_
{

class ViewJPosDialog : public QDialog
{
  Q_OBJECT

public:
  ViewJPosDialog(const arma::vec &jlow_lim, const arma::vec &jup_lim, std::function<arma::vec()> readJointsPosition, QWidget *parent = 0);
  ~ViewJPosDialog();

  void setTitle(const std::string &title_) { this->setWindowTitle(title_.c_str()); }
  void setJointNames(const std::vector<std::string> &joint_names);
  void setUpdateRate(unsigned up_rate_ms) { up_rate_ms_ = up_rate_ms;  emit updateRateChangedSignal(); }

signals:
  void updateRateChangedSignal();
  void updateSliderPosition(int pos);

public slots:
  void launch();
  void stop();

private slots:
  void changeUnits(const QString &units);

private:

  bool run;

  unsigned up_rate_ms_;

  int N_JOINTS;
  arma::vec jlow_lim;
  arma::vec jup_lim;

  std::vector<QLabel *> labels;
  std::vector<QSlider *> sliders;
  std::vector<QLineEdit *> values;
  std::vector<std::string> jnames;
  QComboBox *units_combox;
  QLabel *units_combox_label;

  int slider_max;
  int slider_min;

  enum Units
  {
    DEGREES = 0,
    RAD = 1
  } units;

  std::function<arma::vec()> readJointsPosition;

  void updateDialogThread();

  void updateSliderText(int i);
  void closeEvent(QCloseEvent *event) override;


  double sliderPos2JointPos(int s_pos, int i) const;
  double jointPos2SliderPos(double j_pos, int i) const;

  constexpr static long double pi = 3.14159265359;
};

} // namespace gui_

} // namespace as64_

#endif // GUI_LIB_VIEW_JOINT_POSITION_DIALOG_H
