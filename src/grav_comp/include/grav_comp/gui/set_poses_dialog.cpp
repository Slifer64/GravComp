#include "set_poses_dialog.h"

#include <QDebug>


// =======================================================
// ===============    SetPosesDialog    ==================
// =======================================================

SetPosesDialog::SetPosesDialog(int N_joints, QWidget *parent) : QDialog(parent)
{
    // this->resize(600,500);
    this->setWindowTitle("Set Predefined poses");

    central_widget = this; //new QWidget(this);

    N_JOINTS = N_joints;

    scroll_area_contents = new QWidget; //(central_widget);
    // scroll_area_contents->setGeometry(0,0, 750, 1000);
    // scroll_area_contents->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    // scroll_area_contents->setStyleSheet("border: 1px solid red");
//    poses_widget.push_back(new PoseWidget(N_JOINTS,"pose 1", scroll_area_contents));
//    poses_widget.push_back(new PoseWidget(N_JOINTS,"pose 2", scroll_area_contents));
//    poses_widget.push_back(new PoseWidget(N_JOINTS,"pose 3", scroll_area_contents));

    poses_layout = new QVBoxLayout(scroll_area_contents);
    for (auto it=poses_widget.begin(); it!=poses_widget.end(); it++) poses_layout->addWidget(*it);
    poses_layout->addStretch();

    scroll_area = new QScrollArea;
    scroll_area->setWidget(scroll_area_contents);
    scroll_area->setWidgetResizable(true);

    QFont btn_font = QFont("Ubuntu",15);
    ok_btn = new QPushButton;
    ok_btn->setText("Ok");
    ok_btn->setFont(btn_font);
    ok_btn->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    cancel_btn = new QPushButton;
    cancel_btn->setText("Cancel");
    cancel_btn->setFont(btn_font);
    cancel_btn->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    add_pose_btn = new QPushButton;
    add_pose_btn->setText("Add");
    add_pose_btn->setFont(btn_font);
    add_pose_btn->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    edit_pose_btn = new QPushButton;
    edit_pose_btn->setText("Edit");
    edit_pose_btn->setFont(btn_font);
    edit_pose_btn->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    remove_pose_btn = new QPushButton;
    remove_pose_btn->setText("Remove");
    remove_pose_btn->setFont(btn_font);
    remove_pose_btn->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    QHBoxLayout *btns_layout = new QHBoxLayout;
    btns_layout->addWidget(add_pose_btn);
    btns_layout->addWidget(edit_pose_btn);
    btns_layout->addWidget(remove_pose_btn);
    btns_layout->addStretch();
    //btns_layout->addItem(new QSpacerItem(100,0));
    btns_layout->addWidget(ok_btn,Qt::AlignRight);
    btns_layout->addWidget(cancel_btn,Qt::AlignRight);

    QGridLayout *main_layout = new QGridLayout(central_widget);
    main_layout->addWidget(scroll_area, 0, 0, 1, 3);
    // main_layout->addLayout(poses_layout, 0, 0, 1, 3);
    main_layout->addLayout(btns_layout, 1,0);

    add_pose_dialog = new AddPoseDialog(N_JOINTS, "pose", central_widget);

    QObject::connect(ok_btn, SIGNAL(clicked()), this, SLOT(onOkBtnClicked()));
    QObject::connect(cancel_btn, &QPushButton::clicked, [this](){ this->hide(); } );
    QObject::connect(add_pose_btn, SIGNAL(clicked()), this, SLOT(onAddBtnClicked()));
    QObject::connect(edit_pose_btn, SIGNAL(clicked()), this, SLOT(onEditBtnClicked()));
    QObject::connect(remove_pose_btn, SIGNAL(clicked()), this, SLOT(onRemoveBtnClicked()));
    QObject::connect(add_pose_dialog, SIGNAL(newPoseEntered()), this, SLOT(addNewPose()));
}

SetPosesDialog::~SetPosesDialog()
{}

void SetPosesDialog::launch()
{
  // clear everything
  auto it = poses_widget.begin();
  while (it != poses_widget.end())
  {
      poses_layout->removeWidget(*it);
      delete *it;
      it = poses_widget.erase(it);
  }

  // update
  for (int i=0; i<poses.size(); i++)
  {
    PoseWidget *pose_widg = new PoseWidget(N_JOINTS,"pose " + QString::number(i+1), scroll_area_contents);
    pose_widg->setPose(poses[i]);

    poses_widget.push_back(pose_widg);
    poses_layout->insertWidget(poses_layout->count()-1, pose_widg);
  }

  this->show();
}

void SetPosesDialog::stop()
{
  this->hide();
}

void SetPosesDialog::onOkBtnClicked()
{
  poses.resize(poses_widget.size());

  int i = 0;
  for (auto it=poses_widget.begin(); it!=poses_widget.end(); it++, i++) poses[i] = (*it)->getPose();

  this->hide();
}

void SetPosesDialog::onAddBtnClicked()
{
    add_pose_dialog->show();
}

void SetPosesDialog::onEditBtnClicked()
{
    for (auto it=poses_widget.begin(); it!=poses_widget.end(); it++)
    {
        if ((*it)->isChecked()) (*it)->setReadOnly(false);
    }
}

void SetPosesDialog::onRemoveBtnClicked()
{
    auto it = poses_widget.begin();
    while (it != poses_widget.end())
    {
        if ((*it)->isChecked())
        {
            poses_layout->removeWidget(*it);
            delete *it;
            it = poses_widget.erase(it);
        }
        else it++;
    }

    int i=1;
    for (auto it=poses_widget.begin(); it!=poses_widget.end(); it++) (*it)->setLabel("pose " + QString::number(i++));
}

void SetPosesDialog::addNewPose()
{
    int n_poses = poses_widget.size() + 1;
    PoseWidget *pose_widg = new PoseWidget(N_JOINTS,"pose " + QString::number(n_poses), scroll_area_contents);
    pose_widg->setPose(add_pose_dialog->getPose());

    poses_widget.push_back(pose_widg);
    poses_layout->insertWidget(poses_layout->count()-1, pose_widg);
}


// ======================================================
// ===============    AddPoseDialog    ==================
// ======================================================

AddPoseDialog::AddPoseDialog(int N, const QString &label, QWidget *parent): QDialog(parent)
{
  this->setWindowTitle("Add new pose");

  // ========= pose layout ============

  pose_label = new QLabel;
  pose_label->setFont(QFont("Ubuntu",16));
  pose_label->setText(label);
  pose_label->setFixedSize(pose_label->sizeHint());

  jpos_le.resize(N);
  for (int i=0; i<N; i++)
  {
    jpos_le[i] = new QLineEdit;
    jpos_le[i]->setMinimumSize(75,40);
    jpos_le[i]->setMaximumSize(75,40);
    jpos_le[i]->setAlignment(Qt::AlignCenter);
    jpos_le[i]->setFont(QFont("Ubuntu",14));
    jpos_le[i]->setText("0");
  }
  QHBoxLayout *pose_layout = new QHBoxLayout;
  pose_layout->addWidget(pose_label);
  for (int i=0; i<N; i++) pose_layout->addWidget(jpos_le[i]);
  pose_layout->addStretch();

  // ========= buttons layout ============

  ok_btn = new QPushButton;
  ok_btn->setText("Ok");
  ok_btn->setFixedSize(80,40);
  ok_btn->setFont(QFont("Ubuntu",14));
  cancel_btn = new QPushButton;
  cancel_btn->setText("Cancel");
  cancel_btn->setFixedSize(80,40);
  cancel_btn->setFont(QFont("Ubuntu",14));
  QHBoxLayout *btns_layout = new QHBoxLayout;
  btns_layout->addWidget(ok_btn);
  btns_layout->addWidget(cancel_btn);
  btns_layout->addStretch();

  // ========= main layout ============

  QGridLayout *main_layout = new QGridLayout(this);
  main_layout->addLayout(pose_layout, 0, 0);
  main_layout->addItem(new QSpacerItem(0,25), 1, 0);
  main_layout->addLayout(btns_layout, 2, 0);

  // ========== connections  ============
  QObject::connect(ok_btn, &QPushButton::clicked, [this](){ this->newPoseEntered(); this->hide(); } );
  QObject::connect(cancel_btn, &QPushButton::clicked, [this](){ this->hide(); } );
}

AddPoseDialog::~AddPoseDialog()
{}

arma::vec AddPoseDialog::getPose() const
{
  arma::vec pose(jpos_le.size());
  for (int i=0; i<pose.size(); i++) pose[i] = jpos_le[i]->text().toDouble();

  return pose;
}


// ===================================================
// ===============    PoseWidget    ==================
// ===================================================

PoseWidget::PoseWidget(int N, const QString &label, QWidget *parent): QWidget(parent)
{
  chk_box = new QCheckBox;
  chk_box->setFont(QFont("Ubuntu",16));

  jpos_le.resize(N);
  for (int i=0; i<N; i++)
  {
    jpos_le[i] = new QLineEdit;
    //jpos_le[i]->setMinimumSize(80,40);
    //jpos_le[i]->setMaximumSize(80,40);
    //jpos_le[i]->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    jpos_le[i]->setAlignment(Qt::AlignCenter);
    jpos_le[i]->setFont(QFont("Ubuntu",14));
    jpos_le[i]->setText("0");
  }

  layout = new QHBoxLayout(this);
  layout->addWidget(chk_box);
  for (int i=0; i<N; i++) layout->addWidget(jpos_le[i]);
  layout->addStretch();

  setLabel(label);
  setChecked(false);

  setReadOnly(true);

  QObject::connect(chk_box, &QCheckBox::clicked, [this](){ if (! this->isChecked()) this->setReadOnly(true); });
}

PoseWidget::~PoseWidget()
{}

bool PoseWidget::isChecked() const
{ return chk_box->isChecked(); }

void PoseWidget::setChecked(bool set)
{ chk_box->setChecked(set); }

void PoseWidget::setReadOnly(bool set)
{
  QPalette palette = jpos_le[0]->palette();
  if (set) palette.setColor(QPalette::Base, QColor(250,250,250));
  else palette.setColor(QPalette::Base, QColor(255,255,255));

  for (int i=0; i<jpos_le.size(); i++)
  {
    jpos_le[i]->setReadOnly(set);
    jpos_le[i]->setPalette(palette);
  }
}

void PoseWidget::setPose(const arma::vec &pose)
{
  for (int i=0; i<jpos_le.size() ;i++) jpos_le[i]->setText(QString::number(pose[i]));
}

void PoseWidget::setLabel(const QString &label)
{ chk_box->setText(label); }

arma::vec PoseWidget::getPose() const
{
  arma::vec pose(jpos_le.size());
  for (int i=0; i<pose.size(); i++) pose[i] = jpos_le[i]->text().toDouble();

  return pose;
}