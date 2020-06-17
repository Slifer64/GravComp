#include "mainwindow.h"
#include <grav_comp/gui/utils/utils.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <grav_comp/grav_comp.h>
#include <io_lib/xml_parser.h>

#include <QDebug>

using namespace as64_;

MainWindow::MainWindow(const Robot *robot, GravComp *grav_comp, QWidget *parent): QMainWindow(parent)
{
  this->robot = robot;
  this->grav_comp = grav_comp;

  mode_name[FREEDRIVE] = "FREEDRIVE";
  mode_name[IDLE] = "IDLE";

  //this->resize(400,350);
  this->setWindowTitle("Tool dynamics estimation");

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

  load_wrenchOrient = false;
  save_wrenchOrient = false;
  is_running = true;

  default_data_path = ros::package::getPath("grav_comp") + "/data/";

  mode = FREEDRIVE;
  setMode(IDLE);

  // std::thread([this](){ this->grav_comp->checkRobot();}).detach();
}

MainWindow::~MainWindow()
{

}

// ========================     MODE    ==================================

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

  Robot::Mode robot_mode;
  if (mode==MainWindow::FREEDRIVE) robot_mode = Robot::FREEDRIVE;
  else if (mode==MainWindow::IDLE) robot_mode = Robot::IDLE;

  std::thread([this,robot_mode]()
  {
    this->grav_comp->setMode(robot_mode);
    modeChangedSignal();
  }).detach();

  idle_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");
  freedrive_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");

  this->setEnabled(false);
}

void MainWindow::modeChangedSlot()
{
  this->setEnabled(true);
  updateGUIonModeChanged();
  showInfoMsg("Mode changed to \"" + getModeName() + "\"\n");
}

void MainWindow::updateGUIonModeChanged()
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

// ======================================================================

void MainWindow::createActions()
{
  load_wrenchOrient_ = new ActWidget(this, std::bind(&MainWindow::loadWrenchOrientTriggered, this),
                                     std::bind(&MainWindow::updateGUIonLoadWrenchOrient, this, std::placeholders::_1),
                                    "&Load wrench-orient data...", "Ctrl+D+L", "Loads the wrench-orient data from the file the user specifies.");

  save_wrenchOrient_ = new ActWidget(this, std::bind(&MainWindow::saveWrenchOrientTriggered, this),
                                     std::bind(&MainWindow::updateGUIonSaveWrenchOrient, this, std::placeholders::_1),
                                     "&Save wrench-orient data...", "Ctrl+D+S", "Saves the recorded wrench-orient data to the file specified by the user.");

  load_CoM_ = new ActWidget(this, std::bind(&MainWindow::loadCoMTriggered, this),
                            std::bind(&MainWindow::updateGUIonLoadCoMData, this, std::placeholders::_1),
                            "&Load CoM data...", "Ctrl+L", "Loads the CoM data from the file the user specifies.");

  save_CoM_ = new ActWidget(this, std::bind(&MainWindow::saveCoMTriggered, this),
                            std::bind(&MainWindow::updateGUIonSaveCoM, this, std::placeholders::_1),
                            "&Save CoM data...", "Ctrl+S", "Saves the CoM data to default location.");

  save_as_CoM_ = new ActWidget(this, std::bind(&MainWindow::saveAsCoMTriggered, this),
                            std::bind(&MainWindow::updateGUIonSaveCoM, this, std::placeholders::_1),
                            "&Save CoM data as...", "Shift+Ctrl+S", "Saves the CoM data to a user specified path.");

  load_predef_poses_act = new QAction(tr("Load predefined poses"), this);
  load_predef_poses_act->setStatusTip(tr("Loads predefined poses from a path specified by the user."));
  QObject::connect( load_predef_poses_act, &QAction::triggered, this, &MainWindow::loadPredefPosesTriggered );

  set_predef_poses_act = new QAction(tr("Set predefined poses"), this);
  set_predef_poses_act->setStatusTip(tr("Opens a dialog where you can insert/remove poses at which to record wrench-orient."));
  QObject::connect( set_predef_poses_act, &QAction::triggered, [this](){ this->set_poses_dialog->launch(); } );

  view_wrench_act = new QAction(tr("View wrench"), this);
  view_wrench_act->setStatusTip(tr("Opens a window displaying the compensated tool wrench."));
  QObject::connect( view_wrench_act, &QAction::triggered, [this](){ this->view_wrench_dialog->launch(); } );

  view_est_wrench_act = new QAction(tr("View estimated wrench"), this);
  view_est_wrench_act->setStatusTip(tr("Opens a window displaying the estimated (from the robot) tool wrench."));
  QObject::connect( view_est_wrench_act, &QAction::triggered, [this](){ this->view_est_wrench_dialog->launch(); } );

  view_pose_act = new QAction(tr("View pose"), this);
  view_pose_act->setStatusTip(tr("Opens a window displaying the robot's end-effector pose."));
  QObject::connect( view_pose_act, &QAction::triggered, [this](){ this->view_pose_dialog->launch();} );

  view_joints_act = new QAction(tr("View joints"), this);
  view_joints_act->setStatusTip(tr("Opens a window with sliders displaying the robot's joints position."));
  QObject::connect( view_joints_act, &QAction::triggered, [this](){ this->view_jpos_dialog->launch(); } );

}

void MainWindow::createConnections()
{
  qRegisterMetaType<ExecResultMsg>("ExecResultMsg");

  QObject::connect( this, &MainWindow::robotNotOkSignal, this, &MainWindow::robotNotOkSlot );
  QObject::connect( this, &MainWindow::modeChangedSignal, this, &MainWindow::modeChangedSlot );
}

void MainWindow::createMenus()
{
  menu_bar = new QMenuBar(this);
  this->setMenuBar(menu_bar);
  menu_bar->setNativeMenuBar(false);

  file_menu = menu_bar->addMenu(tr("&File"));
  file_menu->addAction(load_CoM_->act);
  file_menu->addAction(load_wrenchOrient_->act);
  file_menu->addAction(load_predef_poses_act);
  file_menu->addSeparator();
  file_menu->addAction(save_CoM_->act);
  file_menu->addAction(save_as_CoM_->act);
  file_menu->addAction(save_wrenchOrient_->act);

  edit_menu = menu_bar->addMenu(tr("&Edit"));
  edit_menu->addAction(set_predef_poses_act);

  view_menu = menu_bar->addMenu(tr("&View"));
  view_menu->addAction(view_pose_act);
  view_menu->addAction(view_joints_act);
  view_menu->addSeparator();
  view_menu->addAction(view_wrench_act);
  view_menu->addAction(view_est_wrench_act);
  // view_menu->addSeparator();
}

void MainWindow::createWidgets()
{
  QFont font1("Ubuntu", 13, QFont::DemiBold);
  QFont font2("Ubuntu", 15, QFont::DemiBold);

  view_est_wrench_dialog = new ViewWrenchDialog(std::bind(&Robot::getEstimatedTaskWrench, robot), this);
  view_est_wrench_dialog->setWindowTitle("Estimated Tool wrench");
  view_wrench_dialog = new ViewWrenchDialog(std::bind(&Robot::getCompTaskWrench, robot), this);
  view_wrench_dialog->setWindowTitle("Tool wrench");

  view_pose_dialog = new ViewPoseDialog(std::bind(&Robot::getTaskPosition, robot), std::bind(&Robot::getTaskOrientation, robot), this);
  view_jpos_dialog = new ViewJPosDialog(robot->getJointsLowerLimits(), robot->getJointsUpperLimits(), std::bind(&Robot::getJointsPosition, robot), this);
  view_jpos_dialog->setJointNames(robot->getJointNames());
  set_poses_dialog = new SetPosesDialog(robot->getNumOfJoints(), this);

  mode_label = new QLabel;
  mode_label->setText("Robot mode");
  mode_label->setFont(font2);
  mode_label->setStyleSheet("background-color: rgb(250,250,250); color: rgb(0,0,200); font: 75 15pt \"FreeSans\";");
  mode_label->setSizePolicy(QSizePolicy::Preferred,QSizePolicy::Fixed);
  mode_label->setAlignment(Qt::AlignCenter);

  freedrive_btn = new QPushButton;
  freedrive_btn->setText("FREEDRIVE");
  freedrive_btn->setFont(QFont("Ubuntu", 13, QFont::DemiBold));
  QObject::connect( freedrive_btn, &QPushButton::clicked, [this](){ this->setMode(FREEDRIVE);} );

  idle_btn = new QPushButton;
  idle_btn->setText("IDLE");
  idle_btn->setFont(QFont("Ubuntu", 13, QFont::DemiBold));
  QObject::connect( idle_btn, &QPushButton::clicked, [this](){ this->setMode(IDLE);} );


  // ===============================

  emergency_stop_ = new BtnWidget(this, std::bind(&MainWindow::emergencyStopPressed, this),
                     std::bind(&MainWindow::updateGUIonEmergencyStop, this, std::placeholders::_1), "EMERGENCY\n     STOP", font1);
  emergency_stop_->btn->setMinimumSize(80,80);
  emergency_stop_->btn->setIcon(QIcon(":/panic_button_icon"));
  emergency_stop_->btn->setIconSize(QSize(50,50));
  emergency_stop_->btn->setStyleSheet("color:rgb(255,0,0); background-color:rgba(200, 200, 200, 100);");
  emergency_stop_->btn->setFont(QFont("Ubuntu",13,QFont::DemiBold));


  calc_CoM_ = new BtnWidget(this, std::bind(&MainWindow::calcCoMPressed, this),
                            std::bind(&MainWindow::updateGUIonCalcCOMPressed, this, std::placeholders::_1), "Calculate CoM", font1);

  rec_predef_poses_ = new BtnWidget(this, std::bind(&MainWindow::recPredefPosesPressed, this),
                                    std::bind(&MainWindow::updateGUIonRecPredefPoses, this, std::placeholders::_1), "Record predefined poses", font1);

  rec_wrenchQuat_ = new BtnWidget(this, std::bind(&MainWindow::recWrenchQuatPressed, this),
                                  std::bind(&MainWindow::updateGUIonRecWrenchQuat, this, std::placeholders::_1), "record wrench-orient", font1);

  clear_wrenchQuat_ = new BtnWidget(this, std::bind(&MainWindow::clearWrenchQuatDataPressed, this),
                                    std::bind(&MainWindow::updateGUIonClearWrenchQuatData, this, std::placeholders::_1), "clear wrench-orient data", font1);
}

void MainWindow::createLayouts()
{
  mode_layout = new QVBoxLayout;
  mode_layout->addWidget(mode_label);
  mode_layout->addWidget(freedrive_btn);
  mode_layout->addWidget(idle_btn);
  mode_layout->addStretch();

  btns_layout = new QVBoxLayout;
  btns_layout->addWidget(emergency_stop_->btn);
  btns_layout->addWidget(calc_CoM_->btn);
  btns_layout->addWidget(rec_predef_poses_->btn);
  btns_layout->addWidget(rec_wrenchQuat_->btn);
  btns_layout->addWidget(clear_wrenchQuat_->btn);
  btns_layout->addStretch();

  main_layout = new QGridLayout(central_widget);
  main_layout->setSizeConstraint(QLayout::SetFixedSize);
  main_layout->addLayout(mode_layout,0,0);
  main_layout->addItem(new QSpacerItem(30,0,QSizePolicy::Fixed,QSizePolicy::Preferred),0,1);
  main_layout->addLayout(btns_layout,0,2);
  main_layout->addItem(new QSpacerItem(20,10,QSizePolicy::Preferred,QSizePolicy::Fixed),1,0);
}

// ====================    EMERGENCY STOP    ============================

void MainWindow::emergencyStopPressed()
{
  rec_predef_poses_->enabled = false;
  updateGUIonEmergencyStop(false);

  std::thread([this]()
  {
    this->grav_comp->setMode(Robot::IDLE);
    emergency_stop_->ackSignal(ExecResultMsg(ExecResultMsg::WARNING,"Emergency stop activated!"));
  }).detach();



  this->setEnabled(false);
}

void MainWindow::updateGUIonEmergencyStop(bool set)
{
  this->setEnabled(set);

  if (set)
  {
    mode = IDLE;
    freedrive_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");
    idle_btn->setStyleSheet("QPushButton { color: rgb(0, 0, 250); background-color: rgb(0, 255, 0) }");
  }
  else
    {
    idle_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");
    freedrive_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");
  }
}

// ========================     SAVE mass-CoM   ==================================

void MainWindow::saveCoMTriggered()
{
  std::string filename;
  if (!ros::NodeHandle("~").getParam("CoM_filename", filename)) filename="";
  save_data_path = default_data_path + filename;

  updateGUIonSaveCoM(false);

  std::thread([this]()
  {
    ExecResultMsg msg = this->grav_comp->saveCoMData(this->getSaveDataPath());
    save_CoM_->ackSignal(msg);
  }).detach();
}

void MainWindow::saveAsCoMTriggered()
{
  QString save_as_data_path = QFileDialog::getSaveFileName(this, tr("Save mass-CoM Data"), default_data_path.c_str(), "YAML files (*.yaml);;Text files (*.txt);;Binary files (*.bin)");
  if (save_as_data_path.isEmpty()) return;

  save_data_path = save_as_data_path.toStdString();

  updateGUIonSaveCoM(false);

  std::thread([this]()
  {
    ExecResultMsg msg = this->grav_comp->saveCoMData(this->getSaveDataPath());
    save_as_CoM_->ackSignal(msg);
  }).detach();
}

void MainWindow::updateGUIonSaveCoM(bool set)
{
  load_CoM_->setEnabled(set);

  save_CoM_->setEnabled(set);
  save_as_CoM_->setEnabled(set);

  calc_CoM_->setEnabled(set);
}

// ========================     LOAD mass-CoM    ==================================

void MainWindow::loadCoMTriggered()
{
  load_data_path = QFileDialog::getOpenFileName(this, tr("Load CoM Data"), default_data_path.c_str(), "YAML files (*.yaml);;Text files (*.txt);;Binary files (*.bin)").toStdString();
  if (load_data_path.empty()) return;

  updateGUIonLoadCoMData(false);

  std::thread([this]()
  {
    ExecResultMsg msg = this->grav_comp->loadCoMData(this->getLoadDataPath());
    load_CoM_->ackSignal(msg);
  }).detach();
}

void MainWindow::updateGUIonLoadCoMData(bool set)
{
  load_CoM_->setEnabled(set);

  save_CoM_->setEnabled(set);
  save_as_CoM_->setEnabled(set);

  calc_CoM_->setEnabled(set);
}

// ========================     LOAD predef poses    ==================================

void MainWindow::loadPredefPosesTriggered()
{
  std::string path = QFileDialog::getOpenFileName(this, tr("Load poses"), default_data_path.c_str(), "YAML files (*.yaml)").toStdString();
  if (path.empty()) return;

  ExecResultMsg msg;

  // ========  check file format  =========
  std::string suffix;
  FileFormat file_fmt = getFileFormat(path, &suffix);

  if (file_fmt != FileFormat::YAML)
  {
    msg.setType(ExecResultMsg::ERROR);
    msg.setMsg("Only \"yaml\" or \"yml\" format is allowed!\n");
    showMsg(msg);
  }

  // ========  open file  =========
  std::shared_ptr<io_::XmlParser> parser;
  try {
    parser.reset(new io_::XmlParser(path));
  }
  catch(std::exception &e)
  {
    msg.setType(ExecResultMsg::ERROR);
    msg.setMsg("Failed to open file \"" + path + "\".");
    showMsg(msg);
  }

  // ========  read poses  =========
  std::vector<arma::vec> &poses = set_poses_dialog->poses;
  poses.clear();

  std::vector<double> pose;
  int k = 0;
  while (true)
  {
    std::string pose_name = "pose" + (QString::number(++k)).toStdString();
    if (! parser->getParam(pose_name, pose)) break;
    poses.push_back(arma::vec(pose));
  }

  // ========  fill in return message  =========
  std::ostringstream oss;
  oss << "Number of loaded poses = " << poses.size() << "\n";

  msg.setType(ExecResultMsg::INFO);
  msg.setMsg("Poses successfully loaded!\n\n" + oss.str());
  showMsg(msg);
}

// ========================     LOAD wrench-orient data    ==================================

void MainWindow::loadWrenchOrientTriggered()
{
  std::string path = QFileDialog::getOpenFileName(this, tr("Load wrench-orient Data"), default_data_path.c_str(), "YAML files (*.yaml);;Text files (*.txt);;Binary files (*.bin)").toStdString();
  if (path.empty()) return;

  updateGUIonLoadWrenchOrient(false);

  std::thread([this,path]()
  {
    ExecResultMsg msg = this->grav_comp->loadWrenchOrientData(path);
    load_wrenchOrient_->ackSignal(msg);
  }).detach();
}


void MainWindow::updateGUIonLoadWrenchOrient(bool set)
{
  rec_predef_poses_->setEnabled(set);
  rec_wrenchQuat_->setEnabled(set);
  load_wrenchOrient_->setEnabled(set);
  save_wrenchOrient_->setEnabled(set);

  calc_CoM_->setEnabled(set);
}

// ========================     SAVE wrench-orient data    ==================================

void MainWindow::saveWrenchOrientTriggered()
{
  std::string path = QFileDialog::getSaveFileName(this, tr("Save Recorded wrench-orient data"), default_data_path.c_str(), "YAML files (*.yaml);;Text files (*.txt);;Binary files (*.bin)").toStdString();
  if (path.empty()) return;

  updateGUIonSaveWrenchOrient(false);

  std::thread([this,path]()
  {
    ExecResultMsg msg = this->grav_comp->saveWrenchOrientData(path);
    save_wrenchOrient_->ackSignal(msg);
  }).detach();
}


void MainWindow::updateGUIonSaveWrenchOrient(bool set)
{
  rec_predef_poses_->setEnabled(set);
  rec_wrenchQuat_->setEnabled(set);
  load_wrenchOrient_->setEnabled(set);
  save_wrenchOrient_->setEnabled(set);
  clear_wrenchQuat_->setEnabled(set);
}

// =====================   CALC mass-CoM   ================================

void MainWindow::calcCoMPressed()
{
  updateGUIonCalcCOMPressed(false);

  std::thread([this]()
  {
    ExecResultMsg msg = this->grav_comp->calcCoM();
    calc_CoM_->ackSignal(msg);
  }).detach();
}

void MainWindow::updateGUIonCalcCOMPressed(bool set)
{
  calc_CoM_->setEnabled(set);

  clear_wrenchQuat_->setEnabled(set);
  rec_wrenchQuat_->setEnabled(set);
  rec_predef_poses_->setEnabled(set);

  load_CoM_->setEnabled(set);

  save_CoM_->setEnabled(set);
  save_as_CoM_->setEnabled(set);
}

// ========================     CLEAR    ==================================

void MainWindow::clearWrenchQuatDataPressed()
{
  updateGUIonClearWrenchQuatData(false);

  std::thread([this]()
  {
    ExecResultMsg msg = this->grav_comp->clearWrenchQuatData();
    clear_wrenchQuat_->ackSignal(msg);
  }).detach();
}

void MainWindow::updateGUIonClearWrenchQuatData(bool set)
{
  clear_wrenchQuat_->setEnabled(set);

  calc_CoM_->setEnabled(set);
  clear_wrenchQuat_->setEnabled(set);
  rec_wrenchQuat_->setEnabled(set);
  rec_predef_poses_->setEnabled(set);

  load_CoM_->setEnabled(set);

  save_CoM_->setEnabled(set);
  save_as_CoM_->setEnabled(set);
}

// ========================     RECORD    =================================

void MainWindow::recWrenchQuatPressed()
{
  rec_wrenchQuat_->enabled = true;
  updateGUIonRecWrenchQuat(false);

  std::thread([this]()
  {
    ExecResultMsg msg = this->grav_comp->recordCurrentWrenchQuat();
    rec_wrenchQuat_->ackSignal(msg);
  }).detach();
}

void MainWindow::updateGUIonRecWrenchQuat(bool set)
{
  rec_wrenchQuat_->setEnabled(set);

  calc_CoM_->setEnabled(set);
  clear_wrenchQuat_->setEnabled(set);
  rec_wrenchQuat_->setEnabled(set);
  rec_predef_poses_->setEnabled(set);
}

// ==================    RECORD PREDEF POSES    ==========================

void MainWindow::recPredefPosesPressed()
{
  rec_predef_poses_->enabled = true;
  updateGUIonRecPredefPoses(false);

  std::thread thr = std::thread([this]()
  {
    ExecResultMsg msg = this->grav_comp->recPredefPoses();
    rec_predef_poses_->ackSignal(msg);
  });
  makeThreadRT(thr);
  thr.detach();
}

void MainWindow::updateGUIonRecPredefPoses(bool set)
{
  rec_predef_poses_->setEnabled(set);

  calc_CoM_->setEnabled(set);
  clear_wrenchQuat_->setEnabled(set);
  rec_wrenchQuat_->setEnabled(set);
  set_predef_poses_act->setEnabled(set);

  freedrive_btn->setEnabled(set);
  idle_btn->setEnabled(set);

  if (set)
  {
    if (getMode()==FREEDRIVE) freedrive_btn->setStyleSheet("QPushButton { color: rgb(0, 0, 250); background-color: rgb(0, 255, 0); }");
    else if  (getMode()==IDLE) idle_btn->setStyleSheet("QPushButton { color: rgb(0, 0, 250); background-color: rgb(0, 255, 0); }");
  }
  else
  {
    idle_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");
    freedrive_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");
  }

}

// ======================================================================

void MainWindow::closeEvent(QCloseEvent *event)
{
  // reset all flags
  is_running = false;

  std::thread([this](){ this->grav_comp->setMode(Robot::IDLE); }).detach();

  // update_gui_sem.notify(); // unlock possible waits...
  QMainWindow::closeEvent(event);
}

void MainWindow::robotNotOkSlot(const QString &msg)
{
  QMessageBox msg_box;
  msg_box.setText(msg + "\nDo you want to close the program?");
  msg_box.setIcon(QMessageBox::Critical);
  msg_box.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
  msg_box.setModal(true);

  int res = msg_box.exec();

  if (res == QMessageBox::Yes) this->close();
  // else do nothing ...
}
