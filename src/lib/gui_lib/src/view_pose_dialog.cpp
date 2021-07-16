#include <gui_lib/view_pose_dialog.h>

namespace as64_
{

namespace gui_
{

ViewPoseDialog::ViewPoseDialog(std::function<arma::vec()> getPose, QWidget *parent): QDialog(parent)
{
  run = false;

  up_rate_ms_ = 50;

  get_pose_fun_ = getPose;
  if (get_pose_fun_().size() != 7) throw std::runtime_error("[ViewPoseDialog::ViewPoseDialog]: getPose must return a vector of length 7!");

  this->setWindowTitle("Pose");

  // ============  Pose widgets  ================
  QLabel *pos_label = new QLabel("Position");
  pos_label->setStyleSheet("background-color: rgb(245,245,245); color: rgb(0,0,0); font: 75 14pt \"FreeSans\";");
  QLabel *x_label = new QLabel("x");
  QLabel *y_label = new QLabel("y");
  QLabel *z_label = new QLabel("z");
  QLabel *m_label = new QLabel("m");

  x_le = createLineEdit();
  y_le = createLineEdit();
  z_le = createLineEdit();

  QLabel *orient_label = new QLabel("Orientation\n(Quaternion)");
  orient_label->setStyleSheet("background-color: rgb(245,245,245); color: rgb(0,0,0); font: 75 14pt \"FreeSans\";");
  QLabel *scalar_label = new QLabel("scalar");
  QLabel *vector_label = new QLabel("vector");

  qw_le = createLineEdit();
  qx_le = createLineEdit();
  qy_le = createLineEdit();
  qz_le = createLineEdit();

  // ============  Pose layout  ================
  QGridLayout *pose_layout = new QGridLayout;
    pose_layout->addWidget(x_label,0,1, Qt::AlignCenter);
    pose_layout->addWidget(y_label,0,2, Qt::AlignCenter);
    pose_layout->addWidget(z_label,0,3, Qt::AlignCenter);
    pose_layout->addWidget(pos_label,1,0, Qt::AlignCenter);
    pose_layout->addWidget(x_le,1,1);
    pose_layout->addWidget(y_le,1,2);
    pose_layout->addWidget(z_le,1,3);
    pose_layout->addWidget(m_label,1,4);
    pose_layout->addItem(new QSpacerItem(0,20),2,0);
    pose_layout->addWidget(orient_label,3,0,2,1, Qt::AlignCenter);
    pose_layout->addWidget(scalar_label,3,1, Qt::AlignCenter);
    pose_layout->addWidget(vector_label,3,2,1,3, Qt::AlignCenter);
    pose_layout->addWidget(qw_le,4,1);
    pose_layout->addWidget(qx_le,4,2);
    pose_layout->addWidget(qy_le,4,3);
    pose_layout->addWidget(qz_le,4,4);


  // ============  Refresh widgets  ================
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
  QObject::connect(this, &ViewPoseDialog::updateRateChangedSignal, this, [this,refresh_rate_le](){ refresh_rate_le->setText(QString::number(this->up_rate_ms_)); });
  // -----------------------------------------------
  QLabel *refresh_rate_units_lb = new QLabel("ms");
  refresh_rate_units_lb->setStyleSheet("font: 75 14pt;");

  // ============  Refresh layout  ================
  QHBoxLayout *refresh_layout = new QHBoxLayout;
    refresh_layout->addWidget(refresh_rate_lb);
    refresh_layout->addWidget(refresh_rate_le);
    refresh_layout->addWidget(refresh_rate_units_lb);
    refresh_layout->addStretch(0);

  // ============  Main Layout  ================
  QVBoxLayout *main_layout = new QVBoxLayout(this);
  // main_layout->setSizeConstraint(QLayout::SetFixedSize);
  main_layout->addLayout(pose_layout);
  main_layout->addLayout(refresh_layout);

  Qt::ConnectionType connect_type = Qt::AutoConnection;
  MyLineEdit *le_array[] = {x_le, y_le, z_le, qw_le, qx_le, qy_le, qz_le};
  for (int i=0;i<7;i++) QObject::connect(le_array[i], SIGNAL(textChanged(QString)), le_array[i], SLOT(setText(QString)), connect_type);
}

ViewPoseDialog::~ViewPoseDialog()
{
  stop();
}

MyLineEdit *ViewPoseDialog::createLineEdit()
{
  MyLineEdit *le = new MyLineEdit;
  le->setSizeHint(60,30);
  le->setReadOnly(true);
  le->setStyleSheet("font: 75 14pt;");
  le->setAlignment(Qt::AlignCenter);

  return le;
}

void ViewPoseDialog::launch()
{
  if (!run)
  {
    run = true;
    std::thread(&ViewPoseDialog::updateDialogThread, this).detach();
    this->show();
  }
}

void ViewPoseDialog::stop()
{
  if (run)
  {
    run = false;
    this->hide();
  }
}

void ViewPoseDialog::updateDialogThread()
{
  while (run)
  {
    arma::vec pose = get_pose_fun_();
    arma::vec pos = pose.subvec(0,2);
    arma::vec quat = pose.subvec(3,6);

    emit x_le->textChanged(QString::number(pos(0),'f',2));
    emit y_le->textChanged(QString::number(pos(1),'f',2));
    emit z_le->textChanged(QString::number(pos(2),'f',2));

    emit qw_le->textChanged(QString::number(quat(0),'f',2));
    emit qx_le->textChanged(QString::number(quat(1),'f',2));
    emit qy_le->textChanged(QString::number(quat(2),'f',2));
    emit qz_le->textChanged(QString::number(quat(3),'f',2));

    std::this_thread::sleep_for(std::chrono::milliseconds(up_rate_ms_));
  }
}

void ViewPoseDialog::closeEvent(QCloseEvent *event)
{
  stop();
}

} // namespace gui_

} // namespace as64_
