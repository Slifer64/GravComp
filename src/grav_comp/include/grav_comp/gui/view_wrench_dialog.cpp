#include "view_wrench_dialog.h"

#define ViewWrenchDialog_fun_ std::string("[ViewWrenchDialog::") + __func__ + "]: "

ViewWrenchDialog::ViewWrenchDialog(std::function<arma::vec()> readWrench, std::function<arma::mat()> getRelRot, QWidget *parent): QDialog(parent)
{
  run = false;

  this->read_wrench = readWrench;
  this->get_rel_rot = getRelRot;

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
  ref_frame_cmbx->addItem("sensor");
  ref_frame_cmbx->addItem("base");
  ref_frame_cmbx->setMaximumWidth(90);
  //ref_frame_cmbx->setCurrentIndex(0); // degrees
  QObject::connect(ref_frame_cmbx, SIGNAL(currentIndexChanged(const QString &)), this, SLOT(refFrameChangedSlot(const QString &)));
  emit ref_frame_cmbx->currentIndexChanged("sensor");

  QHBoxLayout *ref_frame_layout = new QHBoxLayout;
  ref_frame_layout->addWidget(ref_frame_lb);
  ref_frame_layout->addWidget(ref_frame_cmbx);
  ref_frame_layout->addStretch(0);

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

  QVBoxLayout *main_layout = new QVBoxLayout(this);
  main_layout->addLayout(ref_frame_layout);
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

void ViewWrenchDialog::refFrameChangedSlot(const QString &ref_frame)
{
  if (ref_frame.compare("sensor")==0) get_wrench = std::bind(&ViewWrenchDialog::getLocalWrench, this);
  else if (ref_frame.compare("base")==0) get_wrench = std::bind(&ViewWrenchDialog::getBaseWrench, this);
  else throw std::runtime_error(ViewWrenchDialog_fun_ + "Invalid option...");
}


arma::vec ViewWrenchDialog::getLocalWrench()
{
  arma::vec wrench = read_wrench();
  arma::mat R = get_rel_rot();
  wrench.subvec(0,2) = R.t()*wrench.subvec(0,2);
  wrench.subvec(3,5) = R.t()*wrench.subvec(3,5);
  return wrench;
}

arma::vec ViewWrenchDialog::getBaseWrench()
{
  return read_wrench();
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

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}
}

void ViewWrenchDialog::closeEvent(QCloseEvent *event)
{
  stop();
}
