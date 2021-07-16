#include <gui_lib/view_wrench_dialog.h>

namespace as64_
{

namespace gui_
{

#define ViewWrenchDialog_fun_ std::string("[ViewWrenchDialog::") + __func__ + "]: "

ViewWrenchDialog::ViewWrenchDialog(const std::map< std::string, std::function<arma::vec()> > &wrench_map_, QWidget *parent): QDialog(parent)
{
  init(wrench_map_);
}

ViewWrenchDialog::ViewWrenchDialog(const std::vector<std::function<arma::vec()>> &wrench_funs, const std::vector<std::string> &wrench_fun_names, QWidget *parent): QDialog(parent)
{
  if ( wrench_funs.size() != wrench_fun_names.size() )  throw std::runtime_error(ViewWrenchDialog_fun_ + "Wrench functions and names don't have the same size...\n");
  std::map< std::string, std::function<arma::vec()> > wrench_map_;
  for (int i=0; i<wrench_funs.size(); i++) wrench_map_[wrench_fun_names[i]] = wrench_funs[i];

  init(wrench_map_);
}

ViewWrenchDialog::ViewWrenchDialog(std::function<arma::vec()> readWrench, std::function<arma::mat()> getRelRot, QWidget *parent): QDialog(parent)
{
  std::map< std::string, std::function<arma::vec()> > wrench_map_;
  wrench_map_["base"] = readWrench;
  wrench_map_["sensor"] = [this, readWrench, getRelRot]()
  {
    arma::vec wrench = readWrench();
    arma::mat R = getRelRot();
    wrench.subvec(0,2) = R.t()*wrench.subvec(0,2);
    wrench.subvec(3,5) = R.t()*wrench.subvec(3,5);
    return wrench;
  };

  init(wrench_map_);
}

void ViewWrenchDialog::init(const std::map< std::string, std::function<arma::vec()> > &wrench_map_)
{
  this->wrench_map = wrench_map_;

  run = false;

  up_rate_ms_ = 100;

  this->setWindowTitle("Tool wrench");

  QLabel *pos_label = new QLabel("Force");
  pos_label->setStyleSheet("background-color: rgb(245,245,245); color: rgb(0,0,0); font: 75 14pt \"FreeSans\";");
  QLabel *x_label = new QLabel("x");
  QLabel *y_label = new QLabel("y");
  QLabel *z_label = new QLabel("z");
  QLabel *f_label = new QLabel("N");

  fx_le = createLineEdit();
  fy_le = createLineEdit();
  fz_le = createLineEdit();
  QLabel *t_label = new QLabel("Nm");

  QLabel *orient_label = new QLabel("Torque");
  orient_label->setStyleSheet("background-color: rgb(245,245,245); color: rgb(0,0,0); font: 75 14pt \"FreeSans\";");

  tx_le = createLineEdit();
  ty_le = createLineEdit();
  tz_le = createLineEdit();


  QLabel *ref_frame_lb = new QLabel("Frame:");
  ref_frame_lb->setStyleSheet("font: 75 14pt;");
  ref_frame_lb->setAlignment(Qt::AlignCenter);
  ref_frame_cmbx = new QComboBox;
  for (auto it = wrench_map.begin(); it!=wrench_map.end(); it++) ref_frame_cmbx->addItem(it->first.c_str());
  // ref_frame_cmbx->addItem("base");
  // ref_frame_cmbx->addItem("sensor");
  ref_frame_cmbx->setMaximumWidth(90);
  QObject::connect(ref_frame_cmbx, &QComboBox::currentTextChanged, this, [this](const QString &fun_name)
  { get_wrench = wrench_map.find(fun_name.toStdString())->second; });
  emit ref_frame_cmbx->currentTextChanged(wrench_map.begin()->first.c_str());

  QHBoxLayout *ref_frame_layout = new QHBoxLayout;
  ref_frame_layout->addWidget(ref_frame_lb);
  ref_frame_layout->addWidget(ref_frame_cmbx);
  ref_frame_layout->addStretch(0);


  QLabel *refresh_rate_lb = new QLabel("refresh:");
  refresh_rate_lb->setStyleSheet("font: 75 14pt;");
  // -----------------------------------------------
  QLineEdit *refresh_rate_le = new QLineEdit(QString::number(up_rate_ms_));
  refresh_rate_le->setStyleSheet("font: 75 14pt;");
  refresh_rate_le->setAlignment(Qt::AlignCenter);
  refresh_rate_le->setMaxLength(5);
  //refresh_rate_le->setSizeHint(50,30);
  refresh_rate_le->setMaximumSize(QSize(60,30));
  QObject::connect(refresh_rate_le, &QLineEdit::editingFinished, this, [this,refresh_rate_le]()
  {
    this->up_rate_ms_ = static_cast<unsigned>(refresh_rate_le->text().toDouble());
    refresh_rate_le->setText(QString::number(this->up_rate_ms_));
  });
  QObject::connect(this, &ViewWrenchDialog::updateRateChangedSignal, this, [this,refresh_rate_le](){ refresh_rate_le->setText(QString::number(this->up_rate_ms_)); });
  // -----------------------------------------------
  QLabel *refresh_rate_units_lb = new QLabel("ms");
  refresh_rate_units_lb->setStyleSheet("font: 75 14pt;");

  QHBoxLayout *refresh_rate_layout = new QHBoxLayout;
  ref_frame_layout->addSpacing(30);
  ref_frame_layout->addWidget(refresh_rate_lb);
  ref_frame_layout->addWidget(refresh_rate_le);
  ref_frame_layout->addWidget(refresh_rate_units_lb);
  ref_frame_layout->addStretch(0);

  QHBoxLayout *temp_layout_1 = new QHBoxLayout;
  temp_layout_1->addLayout(ref_frame_layout);
  temp_layout_1->addLayout(refresh_rate_layout);
  temp_layout_1->addStretch(0);

  // -------------------------------------------------------

  QGridLayout *wrench_layout = new QGridLayout;
  // main_layout->setSizeConstraint(QLayout::SetFixedSize);
  wrench_layout->addWidget(x_label,0,1, Qt::AlignCenter);
  wrench_layout->addWidget(y_label,0,2, Qt::AlignCenter);
  wrench_layout->addWidget(z_label,0,3, Qt::AlignCenter);
  wrench_layout->addWidget(pos_label,1,0, Qt::AlignCenter);
  wrench_layout->addWidget(fx_le,1,1);
  wrench_layout->addWidget(fy_le,1,2);
  wrench_layout->addWidget(fz_le,1,3);
  wrench_layout->addWidget(f_label,1,4);
  wrench_layout->addItem(new QSpacerItem(0,20),2,0);
  wrench_layout->addWidget(orient_label,3,0, Qt::AlignCenter);
  wrench_layout->addWidget(tx_le,3,1);
  wrench_layout->addWidget(ty_le,3,2);
  wrench_layout->addWidget(tz_le,3,3);
  wrench_layout->addWidget(t_label,3,4);

  // -------------------------------------------------------

  QVBoxLayout *main_layout = new QVBoxLayout(this);
  main_layout->addLayout(temp_layout_1);
  main_layout->addLayout(wrench_layout);

  Qt::ConnectionType connect_type = Qt::AutoConnection;
  MyLineEdit *le_array[] = {fx_le, fy_le, fz_le, tx_le, ty_le, tz_le};
  for (int i=0;i<6;i++) QObject::connect(le_array[i], SIGNAL(textChanged(QString)), le_array[i], SLOT(setText(QString)), connect_type);
}

ViewWrenchDialog::~ViewWrenchDialog()
{
  stop();
}

MyLineEdit *ViewWrenchDialog::createLineEdit()
{
  MyLineEdit *le = new MyLineEdit;
  le->setSizeHint(60,30);
  le->setReadOnly(true);
  le->setStyleSheet("font: 75 14pt;");
  le->setAlignment(Qt::AlignCenter);

  return le;
}

void ViewWrenchDialog::launch()
{
if (!run)
{
  run = true;
  std::thread(&ViewWrenchDialog::updateDialogThread, this).detach();
  this->show();
}
}

void ViewWrenchDialog::stop()
{
if (run)
{
  run = false;
  this->hide();
}
}

void ViewWrenchDialog::updateDialogThread()
{
  while (run)
  {
    arma::vec wrench = get_wrench();

    emit fx_le->textChanged(QString::number(wrench(0),'f',2));
    emit fy_le->textChanged(QString::number(wrench(1),'f',2));
    emit fz_le->textChanged(QString::number(wrench(2),'f',2));

    emit tx_le->textChanged(QString::number(wrench(3),'f',2));
    emit ty_le->textChanged(QString::number(wrench(4),'f',2));
    emit tz_le->textChanged(QString::number(wrench(5),'f',2));

    std::this_thread::sleep_for(std::chrono::milliseconds(up_rate_ms_));
  }
}

void ViewWrenchDialog::closeEvent(QCloseEvent *event)
{
  stop();
}

} // namespace gui_

} // namespace as64_
