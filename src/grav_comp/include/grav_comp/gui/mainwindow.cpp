#include "mainwindow.h"
#include "utils.h"
#include <ros/package.h>

#include <QDebug>

MainWindow::MainWindow(const Robot *robot, QWidget *parent): QMainWindow(parent)
{
  this->robot = robot;

  mode_name[FREEDRIVE] = "FREEDRIVE";
  mode_name[IDLE] = "IDLE";

  //this->resize(400,350);
  this->setWindowTitle("SE(3) DMP");

  //QToolBar *tool_bar = new QToolBar(this);
  //this->addToolBar(tool_bar);
  status_bar = new QStatusBar(this);
  this->setStatusBar(status_bar);

  central_widget = new QWidget(this);
  this->setCentralWidget(central_widget);

  // ====================================

  createWidgets();

  createLayouts();

  createActions();

  createMenus();

  createConnections();

  calc_CoM = false;
  clear_WrenchQuat_data = false;
  rec_wrenchQuat = false;
  load_data = false;
  save_data = false;
  is_running = true;

  default_data_path = ros::package::getPath("grav_comp") + "/data/";

  mode = FREEDRIVE;
  setMode(IDLE);
}

MainWindow::~MainWindow()
{

}

MainWindow::Mode MainWindow::getMode() const
{
  return mode;
}

QString MainWindow::getModeName() const
{
  return (mode_name.find(getMode()))->second;
}

void MainWindow::setMode(const Mode &m)
{
  if (getMode() == m) return;

  mode = m;

  idle_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");
  freedrive_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");

  this->setEnabled(false);
}

void MainWindow::updateInterfaceOnModeChanged()
{
  switch (getMode())
  {
    case FREEDRIVE:
      idle_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");
      freedrive_btn->setStyleSheet("QPushButton { color: rgb(0, 0, 250); background-color: rgb(0, 255, 0); }");
      break;

    case IDLE:

      freedrive_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");
      idle_btn->setStyleSheet("QPushButton { color: rgb(0, 0, 250); background-color: rgb(0, 255, 0) }");
      break;

  }
}

void MainWindow::createActions()
{
  load_CoM_act = new QAction(tr("&Load CoM data..."), this);
  load_CoM_act->setShortcut(QKeySequence("Ctrl+L"));
  load_CoM_act->setStatusTip(tr("Loads the CoM data from the file the user specifies."));

  save_act = new QAction(tr("&Save CoM data"), this);
  save_act->setShortcut(QKeySequence("Ctrl+S"));
  save_act->setStatusTip(tr("Saves the CoM data to default location."));

  save_as_act = new QAction(tr("Save CoM data as..."), this);
  save_as_act->setShortcut(QKeySequence("Shift+Ctrl+S"));
  save_as_act->setStatusTip(tr("Saves the CoM data to a user specified path."));

  view_wrench_act = new QAction(tr("View wrench"), this);
  view_wrench_act->setStatusTip(tr("Opens a window displaying the compensated tool wrench."));

  view_pose_act = new QAction(tr("View pose"), this);
  view_pose_act->setStatusTip(tr("Opens a window displaying the robot's end-effector pose."));

  view_joints_act = new QAction(tr("View joints"), this);
  view_joints_act->setStatusTip(tr("Opens a window with sliders displaying the robot's joints position."));
}

void MainWindow::createConnections()
{
  QObject::connect( load_CoM_act, &QAction::triggered, this, &MainWindow::loadTriggered );

  QObject::connect( save_act, &QAction::triggered, this, &MainWindow::saveTriggered );
  QObject::connect( save_as_act, &QAction::triggered, this, &MainWindow::saveAsTriggered );

  QObject::connect( view_wrench_act, &QAction::triggered, [this](){ this->view_wrench_dialog->launch(); } );

  QObject::connect( view_pose_act, &QAction::triggered, [this](){ this->view_pose_dialog->launch();} );

  QObject::connect( view_joints_act, &QAction::triggered, [this](){ this->view_jpos_dialog->launch(); } );

  QObject::connect( freedrive_btn, &QPushButton::clicked, [this](){ this->setMode(FREEDRIVE);} );
  QObject::connect( idle_btn, &QPushButton::clicked, [this](){ this->setMode(IDLE);} );

  QObject::connect( calc_CoM_btn, &QPushButton::clicked, this, &MainWindow::calcCoMPressed );
  QObject::connect( rec_wrenchQuat_btn, &QPushButton::clicked, this, &MainWindow::recWrenchQuatPressed );
  QObject::connect( clear_wrenchQuat_btn, &QPushButton::clicked, this, &MainWindow::clearWrenchQuatDataPressed );

  QObject::connect( emergency_stop_btn, &QPushButton::clicked, [this](){ const_cast<Robot *>(this->robot)->setExternalStop(true); this->setEnabled(false); } );

  QObject::connect( this, SIGNAL(notTrainedSignal(const QString &)), this, SLOT(notTrainedSlot(const QString &)) );
  QObject::connect( this, SIGNAL(controllerFinishedSignal(const QString &)), this, SLOT(controllerFinishedSlot(const QString &)) );
  QObject::connect( this, SIGNAL(terminateAppSignal(const QString &)), this, SLOT(terminateAppSlot(const QString &)) );
  QObject::connect( this, SIGNAL(modeChangedSignal()), this, SLOT(modeChangedSlot()) );
  QObject::connect( this, SIGNAL(calcCoMAckSignal(bool , const QString &)), this, SLOT(calcCoMAckSlot(bool , const QString &)) );
  QObject::connect( this, SIGNAL(clearWrenchQuatDataAckSignal(bool , const QString &)), this, SLOT(clearWrenchQuatDataAckSlot(bool , const QString &)) );
  QObject::connect( this, SIGNAL(loadAckSignal(bool , const QString &)), this, SLOT(loadAckSlot(bool , const QString &)) );
  QObject::connect( this, SIGNAL(saveAckSignal(bool , const QString &)), this, SLOT(saveAckSlot(bool , const QString &)) );
  QObject::connect( this, SIGNAL(recWrenchQuatAckSignal(bool , const QString &)), this, SLOT(startPoseAckSlot(bool , const QString &)) );
}

void MainWindow::createMenus()
{
  menu_bar = new QMenuBar(this);
  this->setMenuBar(menu_bar);
  menu_bar->setNativeMenuBar(false);

  file_menu = menu_bar->addMenu(tr("&File"));
  file_menu->addAction(load_CoM_act);
  file_menu->addSeparator();
  file_menu->addAction(save_act);
  file_menu->addAction(save_as_act);

  edit_menu = menu_bar->addMenu(tr("&Edit"));

  view_menu = menu_bar->addMenu(tr("&View"));
  view_menu->addAction(view_pose_act);
  view_menu->addAction(view_joints_act);
  view_menu->addAction(view_wrench_act);
  // view_menu->addSeparator();
}

void MainWindow::createWidgets()
{
  QFont font1("Ubuntu", 13, QFont::DemiBold);
  QFont font2("Ubuntu", 15, QFont::DemiBold);

  view_wrench_dialog = new ViewWrenchDialog(std::bind(&Robot::getCompTaskWrench, robot), this);
  view_pose_dialog = new ViewPoseDialog(std::bind(&Robot::getTaskPosition, robot), std::bind(&Robot::getTaskOrientation, robot), this);
  view_jpos_dialog = new ViewJPosDialog(robot->getJointsLowerLimits(), robot->getJointsUpperLimits(), std::bind(&Robot::getJointsPosition, robot), this);
  view_jpos_dialog->setJointNames(robot->getJointNames());

  mode_label = new QLabel;
  mode_label->setText("Robot mode");
  mode_label->setFont(font2);
  mode_label->setStyleSheet("background-color: rgb(250,250,250); color: rgb(0,0,200); font: 75 15pt \"FreeSans\";");
  mode_label->setSizePolicy(QSizePolicy::Preferred,QSizePolicy::Fixed);
  mode_label->setAlignment(Qt::AlignCenter);

  freedrive_btn = new QPushButton;
  freedrive_btn->setText("FREEDRIVE");
  freedrive_btn->setFont(QFont("Ubuntu", 13, QFont::DemiBold));

  idle_btn = new QPushButton;
  idle_btn->setText("IDLE");
  idle_btn->setFont(QFont("Ubuntu", 13, QFont::DemiBold));


  // ===============================

  emergency_stop_btn = new QPushButton;
  emergency_stop_btn->setText("EMERGENCY\n     STOP");
  emergency_stop_btn->setMinimumSize(80,80);
  emergency_stop_btn->setIcon(QIcon(":/panic_button_icon"));
  emergency_stop_btn->setIconSize(QSize(50,50));
  emergency_stop_btn->setStyleSheet("color:rgb(255,0,0); background-color:rgba(200, 200, 200, 100);");
  emergency_stop_btn->setFont(QFont("Ubuntu",13,QFont::DemiBold));

  calc_CoM_btn = new QPushButton;
  calc_CoM_btn->setText("Calculate CoM");
  calc_CoM_btn->setFont(font1);

  rec_predef_poses_btn = new QPushButton;
  rec_predef_poses_btn->setText("Record predefined poses");
  rec_predef_poses_btn->setFont(font1);

  rec_wrenchQuat_btn = new QPushButton;
  rec_wrenchQuat_btn->setText("record wrench-orient");
  rec_wrenchQuat_btn->setFont(font1);

  clear_wrenchQuat_btn = new QPushButton;
  clear_wrenchQuat_btn->setText("clear wrench-orient data");
  clear_wrenchQuat_btn->setFont(font1);
}

void MainWindow::createLayouts()
{
  mode_layout = new QVBoxLayout;
  mode_layout->addWidget(mode_label);
  mode_layout->addWidget(freedrive_btn);
  mode_layout->addWidget(idle_btn);
  mode_layout->addStretch();

  btns_layout = new QVBoxLayout;
  btns_layout->addWidget(emergency_stop_btn);
  btns_layout->addWidget(calc_CoM_btn);
  btns_layout->addWidget(rec_predef_poses_btn);
  btns_layout->addWidget(rec_wrenchQuat_btn);
  btns_layout->addWidget(clear_wrenchQuat_btn);
  btns_layout->addStretch();

  main_layout = new QGridLayout(central_widget);
  main_layout->setSizeConstraint(QLayout::SetFixedSize);
  main_layout->addLayout(mode_layout,0,0);
  main_layout->addItem(new QSpacerItem(30,0,QSizePolicy::Fixed,QSizePolicy::Preferred),0,1);
  main_layout->addLayout(btns_layout,0,2);
  main_layout->addItem(new QSpacerItem(20,10,QSizePolicy::Preferred,QSizePolicy::Fixed),1,0);
}

void MainWindow::loadTriggered()
{
  load_data_path = QFileDialog::getOpenFileName(this, tr("Load CoM Data"), default_data_path.c_str(), "Binary files (*.bin);;Text files (*.txt);;YAML files (*.yaml)").toStdString();
  if (load_data_path.empty()) return;

  load_data = true;
  updateInterfaceOnLoadData();
}

void MainWindow::saveTriggered()
{
  save_data_path = default_data_path + "exec_data.bin";
  save_data = true;
  updateInterfaceOnSaveData();
}

void MainWindow::saveAsTriggered()
{
  QString save_as_data_path = QFileDialog::getSaveFileName(this, tr("Save Recorded Data"), default_data_path.c_str(), "Binary files (*.bin);;Text files (*.txt);;YAML files (*.yaml)");
  if (save_as_data_path.isEmpty()) return;

  save_data_path = save_as_data_path.toStdString();
  save_data = true;
  updateInterfaceOnSaveData();
}

void MainWindow::loadAckSlot(bool success, const QString &msg)
{
  load_data = false;

  success ? showInfoMsg(msg) : showErrorMsg(msg);

  update_gui_sem.notify();

  updateInterfaceOnLoadData();
}

void MainWindow::saveAckSlot(bool success, const QString &msg)
{
  save_data = false;

  success ? showInfoMsg(msg) : showErrorMsg(msg);

  update_gui_sem.notify();

  updateInterfaceOnSaveData();
}

void MainWindow::updateInterfaceOnSaveData()
{
  bool set = !save_data();

  load_CoM_act->setEnabled(set);

  save_act->setEnabled(set);
  save_as_act->setEnabled(set);
}

void MainWindow::updateInterfaceOnLoadData()
{
  bool set = !load_data();

  load_CoM_act->setEnabled(set);

  save_act->setEnabled(set);
  save_as_act->setEnabled(set);
}

void MainWindow::calcCoMPressed()
{
  calc_CoM = true;
  updateInterfaceOnCalcCOMPressed();
}

void MainWindow::updateInterfaceOnCalcCOMPressed()
{
  bool set = !calc_CoM();

  freedrive_btn->setEnabled(set);
  idle_btn->setEnabled(set);
  if (set)
  {
    if (getMode()==FREEDRIVE) freedrive_btn->setStyleSheet("QPushButton { color: rgb(0, 0, 250); background-color: rgb(0, 255, 0); }");
    else if (getMode()==IDLE) idle_btn->setStyleSheet("QPushButton { color: rgb(0, 0, 250); background-color: rgb(0, 255, 0); }");
  }
  else
  {
    freedrive_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");
    idle_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");
  }

  calc_CoM_btn->setEnabled(set);

  clear_wrenchQuat_btn->setEnabled(set);
  rec_wrenchQuat_btn->setEnabled(set);

  load_CoM_act->setEnabled(set);

  save_act->setEnabled(set);
  save_as_act->setEnabled(set);
}

void MainWindow::calcCoMAckSlot(bool success, const QString &msg)
{
  calc_CoM = false;

  success ? showInfoMsg(msg) : showErrorMsg(msg);

  update_gui_sem.notify();

  updateInterfaceOnCalcCOMPressed();
}

void MainWindow::clearWrenchQuatDataPressed()
{
  clear_WrenchQuat_data = true;
  updateInterfaceOnClearWrenchQuatData();
}

void MainWindow::clearWrenchQuatDataAckSlot(bool success, const QString &msg)
{
  clear_WrenchQuat_data = false;

  success ? showInfoMsg(msg) : showErrorMsg(msg);

  update_gui_sem.notify();

  updateInterfaceOnClearWrenchQuatData();
}

void MainWindow::updateInterfaceOnClearWrenchQuatData()
{
  bool set = !clear_WrenchQuat_data();

  freedrive_btn->setEnabled(set);
  idle_btn->setEnabled(set);
  if (set)
  {
    if (getMode()==FREEDRIVE) freedrive_btn->setStyleSheet("QPushButton { color: rgb(0, 0, 250); background-color: rgb(0, 255, 0); }");
    else if (getMode()==IDLE) idle_btn->setStyleSheet("QPushButton { color: rgb(0, 0, 250); background-color: rgb(0, 255, 0); }");
  }
  else
  {
    freedrive_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");
    idle_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");
  }

  clear_wrenchQuat_btn->setEnabled(set);
  rec_wrenchQuat_btn->setEnabled(set);

  load_CoM_act->setEnabled(set);

  save_act->setEnabled(set);
  save_as_act->setEnabled(set);
}

void MainWindow::recWrenchQuatPressed()
{
  rec_wrenchQuat = true;
  rec_wrenchQuat_btn->setEnabled(false);
}

void MainWindow::startPoseAckSlot(bool success, const QString &msg)
{
  rec_wrenchQuat = false;

  success ? showInfoMsg(msg) : showErrorMsg(msg);

  update_gui_sem.notify();

  rec_wrenchQuat_btn->setEnabled(true);
}

void MainWindow::closeEvent(QCloseEvent *event)
{
  is_running = false;
  const_cast<Robot *>(robot)->setExternalStop(true);
  update_gui_sem.notify(); // unlock possible waits...
  QMainWindow::closeEvent(event);
}

void MainWindow::controllerFinishedSlot(const QString &msg)
{
  showInfoMsg(msg);
  setMode(IDLE);
}

void MainWindow::notTrainedSlot(const QString &msg)
{
  showErrorMsg(msg);
  setMode(IDLE);
}

void MainWindow::terminateAppSlot(const QString &msg)
{
  showErrorMsg(msg);

  QMessageBox msg_box;
  msg_box.setText(msg);
  msg_box.setIcon(QMessageBox::Critical);
  msg_box.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
  msg_box.setModal(true);

  int res = msg_box.exec();

  if (res == QMessageBox::Yes) this->close();
  // else do nothing ...
}

void MainWindow::modeChangedSlot()
{
  this->setEnabled(true);
  updateInterfaceOnModeChanged();
  showInfoMsg("Mode changed to \"" + getModeName() + "\"\n");
}
