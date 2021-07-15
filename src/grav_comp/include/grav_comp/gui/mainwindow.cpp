#include "mainwindow.h"
#include "utils.h"

#include <ros/ros.h>
#include <ros/package.h>

#include <grav_comp/grav_comp.h>
#include <io_lib/xml_parser.h>

#include <QDebug>

using namespace as64_;

MainWindow::MainWindow(const rw_::Robot *robot, GravComp *grav_comp, QWidget *parent): QMainWindow(parent)
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
  
  qRegisterMetaType<ExecResultMsg>("ExecResultMsg");

  QObject::connect( this, &MainWindow::robotNotOkSignal, this, &MainWindow::robotNotOkSlot );
  QObject::connect( this, &MainWindow::modeChangedSignal, this, &MainWindow::modeChangedSlot );
  QObject::connect( this, &MainWindow::emergencyStopAckSignal, this, &MainWindow::emergencyStopAckSlot );
  QObject::connect( this, &MainWindow::calcCoMAckSignal, this, &MainWindow::calcCoMAckSlot );
  QObject::connect( this, &MainWindow::clearWrenchQuatDataAckSignal, this, &MainWindow::clearWrenchQuatDataAckSlot );
  QObject::connect( this, &MainWindow::loadAckSignal, this, &MainWindow::loadAckSlot );
  QObject::connect( this, &MainWindow::saveAckSignal, this, &MainWindow::saveAckSlot );
  QObject::connect( this, &MainWindow::loadWrenchOrientAckSignal, this, &MainWindow::loadWrenchOrientAckSlot );
  QObject::connect( this, &MainWindow::saveWrenchOrientAckSignal, this, &MainWindow::saveWrenchOrientAckSlot );
  QObject::connect( this, &MainWindow::recWrenchQuatAckSignal, this, &MainWindow::recWrenchQuatAckSlot );
  QObject::connect( this, &MainWindow::recPredefPosesAckSignal, this, &MainWindow::recPredefPosesAckSlot );

  QObject::connect( this, SIGNAL(closeSignal()), this, SLOT(close()) );

  calc_CoM = false;
  clear_WrenchQuat_data = false;
  rec_wrenchQuat = false;
  rec_predef_poses = false;
  load_data = false;
  save_data = false;
  load_wrenchOrient = false;
  save_wrenchOrient = false;
  emergency_stop = false;
  is_running = true;

  default_data_path = ros::package::getPath("grav_comp") + "/data/";

  mode = FREEDRIVE;
  setMode(IDLE);

  // std::thread([this](){ this->grav_comp->checkRobot();}).detach();
}

MainWindow::~MainWindow()
{ }

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

  rw_::Mode robot_mode;
  if (mode==MainWindow::FREEDRIVE) robot_mode = rw_::FREEDRIVE;
  else if (mode==MainWindow::IDLE) robot_mode = rw_::IDLE;

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
  load_predef_poses_act = new QAction(tr("Load predefined poses"), this);
  load_predef_poses_act->setStatusTip(tr("Loads predefined poses from a path specified by the user."));
  QObject::connect( load_predef_poses_act, &QAction::triggered, this, &MainWindow::loadPredefPosesTriggered );

  load_wrenchOrient_act = new QAction(tr("&Load wrench-orient data..."), this);
  load_wrenchOrient_act->setShortcut(QKeySequence("Ctrl+D+L"));
  load_wrenchOrient_act->setStatusTip(tr("Loads the wrench-orient data from the file the user specifies."));
  QObject::connect( load_wrenchOrient_act, &QAction::triggered, this, &MainWindow::loadWrenchOrientTriggered );
  
  save_wrenchOrient_act = new QAction(tr("&Save wrench-orient data..."), this);
  save_wrenchOrient_act->setShortcut(QKeySequence("Ctrl+D+S"));
  save_wrenchOrient_act->setStatusTip(tr("Saves the recorded wrench-orient data to the file specified by the user."));
  QObject::connect( save_wrenchOrient_act, &QAction::triggered, this, &MainWindow::saveWrenchOrientTriggered );

  load_CoM_act = new QAction(tr("&Load CoM data..."), this);
  load_CoM_act->setShortcut(QKeySequence("Ctrl+L"));
  load_CoM_act->setStatusTip(tr("Loads the CoM data from the file the user specifies."));
  QObject::connect( load_CoM_act, &QAction::triggered, this, &MainWindow::loadTriggered );

  save_act = new QAction(tr("&Save CoM data"), this);
  save_act->setShortcut(QKeySequence("Ctrl+S"));
  save_act->setStatusTip(tr("Saves the CoM data to default location."));
  QObject::connect( save_act, &QAction::triggered, this, &MainWindow::saveTriggered );

  save_as_act = new QAction(tr("Save CoM data as..."), this);
  save_as_act->setShortcut(QKeySequence("Shift+Ctrl+S"));
  save_as_act->setStatusTip(tr("Saves the CoM data to a user specified path."));
  QObject::connect( save_as_act, &QAction::triggered, this, &MainWindow::saveAsTriggered );

  set_predef_poses_act = new QAction(tr("Set predefined poses"), this);
  set_predef_poses_act->setStatusTip(tr("Opens a dialog where you can insert/remove poses at which to record wrench-orient."));
  QObject::connect( set_predef_poses_act, &QAction::triggered, [this](){ this->set_poses_dialog->launch(); } );

  bias_FTsensor_act = new QAction(tr("Bias FT-sensor"), this);
  bias_FTsensor_act->setStatusTip(tr("Bias the FT-sensor."));
  QObject::connect( bias_FTsensor_act, &QAction::triggered, [this](){ showMsg( grav_comp->biasFTsensor() ); } );

  
  view_wrench_act = new QAction(tr("View wrench"), this);
  view_wrench_act->setStatusTip(tr("Opens a window displaying the tool wrench."));
  QObject::connect( view_wrench_act, &QAction::triggered, [this](){ this->view_wrench_dialog->launch(); } );

  view_compWrench_act = new QAction(tr("View compensated wrench"), this);
  view_compWrench_act->setStatusTip(tr("Opens a window displaying the compensated tool wrench."));
  QObject::connect( view_compWrench_act, &QAction::triggered, [this](){ this->view_compWrench_dialog->launch(); } );

  view_pose_act = new QAction(tr("View pose"), this);
  view_pose_act->setStatusTip(tr("Opens a window displaying the robot's end-effector pose."));
  QObject::connect( view_pose_act, &QAction::triggered, [this](){ this->view_pose_dialog->launch();} );

  view_joints_act = new QAction(tr("View joints"), this);
  view_joints_act->setStatusTip(tr("Opens a window with sliders displaying the robot's joints position."));
  QObject::connect( view_joints_act, &QAction::triggered, [this](){ this->view_jpos_dialog->launch(); } );
}

void MainWindow::createMenus()
{
  menu_bar = new QMenuBar(this);
  this->setMenuBar(menu_bar);
  menu_bar->setNativeMenuBar(false);

  file_menu = menu_bar->addMenu(tr("&File"));
  file_menu->addAction(load_CoM_act);
  file_menu->addAction(load_wrenchOrient_act);
  file_menu->addAction(load_predef_poses_act);
  file_menu->addSeparator();
  file_menu->addAction(save_act);
  file_menu->addAction(save_as_act);
  file_menu->addAction(save_wrenchOrient_act);

  edit_menu = menu_bar->addMenu(tr("&Edit"));
  edit_menu->addAction(set_predef_poses_act);
  edit_menu->addAction(bias_FTsensor_act);

  view_menu = menu_bar->addMenu(tr("&View"));
  view_menu->addAction(view_pose_act);
  view_menu->addAction(view_joints_act);
  view_menu->addSeparator();
  view_menu->addAction(view_wrench_act);
  view_menu->addAction(view_compWrench_act);
  view_menu->addSeparator();
  // --------------------------------------------
  QMenu *rviz_menu = new QMenu("rviz");

  view_ee_tf_act = new QAction("robot ee tf", this);
  view_ee_tf_act->setCheckable(true);
  view_ee_tf_act->setChecked(false);
  QObject::connect( view_ee_tf_act, &QAction::triggered, this, [this]()
  {
    static bool checked_ = false;
    checked_ = !checked_;
    view_ee_tf_act->setChecked(checked_);
    if (checked_) grav_comp->ee_tf_pub->start();
    else grav_comp->ee_tf_pub->stop();
  });
  rviz_menu->addAction(view_ee_tf_act);

  view_CoM_tf_act = new QAction("CoM tf", this);
  view_CoM_tf_act->setCheckable(true);
  view_CoM_tf_act->setChecked(false);
  QObject::connect( view_CoM_tf_act, &QAction::triggered, this, [this]()
  {
    static bool checked_ = false;
    checked_ = !checked_;
    view_CoM_tf_act->setChecked(checked_);
    if (checked_) grav_comp->com_tf_pub->start(1000);
    else grav_comp->com_tf_pub->stop();
  });
  rviz_menu->addAction(view_CoM_tf_act);

  view_menu->addMenu(rviz_menu);
  // --------------------------------------------

  // view_menu->addSeparator();
}

void MainWindow::createWidgets()
{
  QFont font1("Ubuntu", 13, QFont::DemiBold);
  QFont font2("Ubuntu", 15, QFont::DemiBold);

  view_wrench_dialog = new gui_::ViewWrenchDialog(std::bind(&rw_::Robot::getTaskWrench, robot), std::bind(&rw_::Robot::getTaskRotMat, robot), this);
  view_wrench_dialog->setTitle("Tool wrench");

  view_compWrench_dialog = new gui_::ViewWrenchDialog(std::bind(&rw_::Robot::getCompTaskWrench, robot), std::bind(&rw_::Robot::getTaskRotMat, robot), this);
  view_compWrench_dialog->setTitle("Compensated Tool wrench");

  auto getTaskPoseFun = [this]()
  {
    arma::vec pose = arma::join_vert(robot->getTaskPosition(), robot->getTaskOrientation());
    return pose;
  };
  view_pose_dialog = new gui_::ViewPoseDialog( getTaskPoseFun, this);
  view_pose_dialog->setTitle("Tool pose");

  view_jpos_dialog = new gui_::ViewJPosDialog(robot->getJointPosLowLim(), robot->getJointPosUpperLim(), std::bind(&rw_::Robot::getJointsPosition, robot), this);
  view_jpos_dialog->setJointNames(robot->getJointNames());
  view_jpos_dialog->setTitle("Robot joints position");
  
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

  emergency_stop_btn = new QPushButton;
  emergency_stop_btn->setText("EMERGENCY\n     STOP");
  emergency_stop_btn->setMinimumSize(80,80);
  emergency_stop_btn->setIcon(QIcon(":/panic_button_icon"));
  emergency_stop_btn->setIconSize(QSize(50,50));
  emergency_stop_btn->setStyleSheet("color:rgb(255,0,0); background-color:rgba(200, 200, 200, 100);");
  emergency_stop_btn->setFont(QFont("Ubuntu",13,QFont::DemiBold));
  QObject::connect( emergency_stop_btn, &QPushButton::clicked, this, &MainWindow::emergencyStopPressed);

  calc_CoM_btn = new QPushButton;
  calc_CoM_btn->setText("Calculate CoM");
  calc_CoM_btn->setFont(font1);
  QObject::connect( calc_CoM_btn, &QPushButton::clicked, this, &MainWindow::calcCoMPressed );

  rec_predef_poses_btn = new QPushButton;
  rec_predef_poses_btn->setText("Record predefined poses");
  rec_predef_poses_btn->setFont(font1);
  QObject::connect( rec_predef_poses_btn, &QPushButton::clicked, this, &MainWindow::recPredefPosesPressed );
  
  rec_wrenchQuat_btn = new QPushButton;
  rec_wrenchQuat_btn->setText("record wrench-orient");
  rec_wrenchQuat_btn->setFont(font1);
  QObject::connect( rec_wrenchQuat_btn, &QPushButton::clicked, this, &MainWindow::recWrenchQuatPressed );

  clear_wrenchQuat_btn = new QPushButton;
  clear_wrenchQuat_btn->setText("clear wrench-orient data");
  clear_wrenchQuat_btn->setFont(font1);
  QObject::connect( clear_wrenchQuat_btn, &QPushButton::clicked, this, &MainWindow::clearWrenchQuatDataPressed );
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

// ====================    EMERGENCY STOP    ============================

void MainWindow::emergencyStopPressed()
{
  emergency_stop = true;
  rec_predef_poses = false;

  std::thread([this]()
  {
    this->grav_comp->setMode(rw_::IDLE);
    emergencyStopAckSignal();
  }).detach();

  idle_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");
  freedrive_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");

  this->setEnabled(false);
}

void MainWindow::emergencyStopAckSlot()
{
  this->setEnabled(true);

  mode = IDLE;
  freedrive_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");
  idle_btn->setStyleSheet("QPushButton { color: rgb(0, 0, 250); background-color: rgb(0, 255, 0) }");

  showInfoMsg("The robot stopped!\n");
}

// ========================     SAVE mass-CoM   ==================================

void MainWindow::saveTriggered()
{
  std::string filename;
  if (!ros::NodeHandle("~").getParam("CoM_filename", filename)) filename="";
  save_data_path = default_data_path + filename;
  save_data = true;
  updateGUIonSaveData();

  std::thread([this]()
  {
    ExecResultMsg msg = this->grav_comp->saveCoMData(this->getSaveDataPath());
    saveAckSignal(msg);
  }).detach();
}

void MainWindow::saveAsTriggered()
{
  QString save_as_data_path = QFileDialog::getSaveFileName(this, tr("Save mass-CoM Data"), default_data_path.c_str(), "YAML files (*.yaml);;Text files (*.txt);;Binary files (*.bin)");
  if (save_as_data_path.isEmpty()) return;

  save_data_path = save_as_data_path.toStdString();
  save_data = true;
  updateGUIonSaveData();

  std::thread([this]()
  {
    ExecResultMsg msg = this->grav_comp->saveCoMData(this->getSaveDataPath());
    saveAckSignal(msg);
  }).detach();
}

void MainWindow::saveAckSlot(const ExecResultMsg &msg)
{
  save_data = false;
  showMsg(msg);
  updateGUIonSaveData();
}

void MainWindow::updateGUIonSaveData()
{
  bool set = !save_data();

  load_CoM_act->setEnabled(set);

  save_act->setEnabled(set);
  save_as_act->setEnabled(set);

  calc_CoM_btn->setEnabled(set);
}

// ========================     LOAD mass-CoM    ==================================

void MainWindow::loadTriggered()
{
  load_data_path = QFileDialog::getOpenFileName(this, tr("Load CoM Data"), default_data_path.c_str(), "YAML files (*.yaml);;Text files (*.txt);;Binary files (*.bin)").toStdString();
  if (load_data_path.empty()) return;

  // std::string filename;
  // if (!ros::NodeHandle("~").getParam("CoM_filename", filename)) filename="";
  // load_data_path = default_data_path + filename;
  load_data = true;
  updateGUIonLoadData();

  std::thread([this]()
  {
    ExecResultMsg msg = this->grav_comp->loadCoMData(this->getLoadDataPath());
    loadAckSignal(msg);
  }).detach();
}

void MainWindow::loadAckSlot(const ExecResultMsg &msg)
{
  load_data = false;
  showMsg(msg);
  updateGUIonLoadData();
}

void MainWindow::updateGUIonLoadData()
{
  bool set = !load_data();

  load_CoM_act->setEnabled(set);

  save_act->setEnabled(set);
  save_as_act->setEnabled(set);

  calc_CoM_btn->setEnabled(set);
}

// ========================     LOAD predef poses    ==================================

void MainWindow::loadPredefPosesTriggered()
{
  std::string path = QFileDialog::getOpenFileName(this, tr("Load poses"), default_data_path.c_str(), "YAML files (*.yaml)").toStdString();
  if (path.empty()) return;

  // ========  check file format  =========
  //std::string suffix;
  // FileFormat file_fmt = getFileFormat(path, &suffix);

//  if (file_fmt != FileFormat::YAML)
//  {
//    showMsg(ExecResultMsg(ExecResultMsg::ERROR, "Only \"yaml\" or \"yml\" format is allowed!\n"));
//    return;
//  }

  // ========  open file  =========
  std::shared_ptr<io_::XmlParser> parser;
  try{
    parser.reset(new io_::XmlParser(path));
  }
  catch(std::exception &e)
  { showMsg(ExecResultMsg(ExecResultMsg::ERROR, "Failed to open file \"" + path + "\".")); }

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
  showMsg(ExecResultMsg(ExecResultMsg::INFO, "Poses successfully loaded!\n\n" + oss.str()));
}

// ========================     LOAD wrench-orient data    ==================================

void MainWindow::loadWrenchOrientTriggered()
{
  std::string path = QFileDialog::getOpenFileName(this, tr("Load wrench-orient Data"), default_data_path.c_str(), "YAML files (*.yaml);;Text files (*.txt);;Binary files (*.bin)").toStdString();
  // std::string path;
  // if (!ros::NodeHandle("~").getParam("wrench_orient_data_filename", path)) path="";
  // path = default_data_path + path;

  if (path.empty()) return;

  load_wrenchOrient = true;
  updateGUIonLoadWrenchOrient();

  std::thread([this,path]()
  {
    ExecResultMsg msg = this->grav_comp->loadWrenchOrientData(path);
    loadWrenchOrientAckSignal(msg);
  }).detach();
}

void MainWindow::loadWrenchOrientAckSlot(const ExecResultMsg &msg)
{
  load_wrenchOrient = false;
  showMsg(msg);
  updateGUIonLoadWrenchOrient();
}

void MainWindow::updateGUIonLoadWrenchOrient()
{
  bool set = !load_wrenchOrient();

  rec_predef_poses_btn->setEnabled(set);
  rec_wrenchQuat_btn->setEnabled(set);
  load_wrenchOrient_act->setEnabled(set);
  save_wrenchOrient_act->setEnabled(set);

  calc_CoM_btn->setEnabled(set);
}

// ========================     SAVE wrench-orient data    ==================================

void MainWindow::saveWrenchOrientTriggered()
{
  std::string path = QFileDialog::getSaveFileName(this, tr("Save Recorded wrench-orient data"), default_data_path.c_str(), "YAML files (*.yaml);;Text files (*.txt);;Binary files (*.bin)").toStdString();
  // std::string path;
  // if (!ros::NodeHandle("~").getParam("wrench_orient_data_filename", path)) path="";
  // path = default_data_path + path;

  if (path.empty()) return;

  save_wrenchOrient = true;
  updateGUIonSaveWrenchOrient();

  std::thread([this,path]()
  {
    ExecResultMsg msg = this->grav_comp->saveWrenchOrientData(path);
    saveWrenchOrientAckSignal(msg);
  }).detach();
}

void MainWindow::saveWrenchOrientAckSlot(const ExecResultMsg &msg)
{
  save_wrenchOrient = false;
  showMsg(msg);
  updateGUIonSaveWrenchOrient();
}

void MainWindow::updateGUIonSaveWrenchOrient()
{
  bool set = !save_wrenchOrient();

  rec_predef_poses_btn->setEnabled(set);
  rec_wrenchQuat_btn->setEnabled(set);
  load_wrenchOrient_act->setEnabled(set);
  save_wrenchOrient_act->setEnabled(set);
  clear_wrenchQuat_btn->setEnabled(set);
}


// =====================   CALC mass-CoM   ================================

void MainWindow::calcCoMPressed()
{
  calc_CoM = true;
  updateGUIonCalcCOMPressed();

  std::thread([this]()
  {
    ExecResultMsg msg = this->grav_comp->calcCoM();
    calcCoMAckSignal(msg);
  }).detach();
}

void MainWindow::calcCoMAckSlot(const ExecResultMsg &msg)
{
  calc_CoM = false;
  showMsg(msg);
  updateGUIonCalcCOMPressed();
}

void MainWindow::updateGUIonCalcCOMPressed()
{
  bool set = !calc_CoM();

  calc_CoM_btn->setEnabled(set);

  clear_wrenchQuat_btn->setEnabled(set);
  rec_wrenchQuat_btn->setEnabled(set);
  rec_predef_poses_btn->setEnabled(set);

  load_CoM_act->setEnabled(set);

  save_act->setEnabled(set);
  save_as_act->setEnabled(set);
}

// ========================     CLEAR    ==================================

void MainWindow::clearWrenchQuatDataPressed()
{
  clear_WrenchQuat_data = true;
  updateGUIonClearWrenchQuatData();

  std::thread([this]()
  {
    ExecResultMsg msg = this->grav_comp->clearWrenchQuatData();
    clearWrenchQuatDataAckSignal(msg);
  }).detach();
}

void MainWindow::clearWrenchQuatDataAckSlot(const ExecResultMsg &msg)
{
  clear_WrenchQuat_data = false;
  showMsg(msg);
  updateGUIonClearWrenchQuatData();
}

void MainWindow::updateGUIonClearWrenchQuatData()
{
  bool set = !clear_WrenchQuat_data();

  calc_CoM_btn->setEnabled(set);
  clear_wrenchQuat_btn->setEnabled(set);
  rec_wrenchQuat_btn->setEnabled(set);
  rec_predef_poses_btn->setEnabled(set);

  load_CoM_act->setEnabled(set);

  save_act->setEnabled(set);
  save_as_act->setEnabled(set);
}

// ========================     RECORD    =================================

void MainWindow::recWrenchQuatPressed()
{
  rec_wrenchQuat = true;
  rec_wrenchQuat_btn->setEnabled(false);
  updateGUIonRecWrenchQuat();

  std::thread([this]()
  {
    ExecResultMsg msg = this->grav_comp->recordCurrentWrenchQuat();
    recWrenchQuatAckSignal(msg);
  }).detach();
}

void MainWindow::recWrenchQuatAckSlot(const ExecResultMsg &msg)
{
  rec_wrenchQuat = false;
  showMsg(msg);
  updateGUIonRecWrenchQuat();
}

void MainWindow::updateGUIonRecWrenchQuat()
{
  bool set = !rec_wrenchQuat();

  calc_CoM_btn->setEnabled(set);
  clear_wrenchQuat_btn->setEnabled(set);
  rec_wrenchQuat_btn->setEnabled(set);
  rec_predef_poses_btn->setEnabled(set);
}

// ==================    RECORD PREDEF POSES    ==========================

void MainWindow::recPredefPosesPressed()
{
  rec_predef_poses = true;
  rec_predef_poses_btn->setEnabled(false);
  updateGUIonRecPredefPoses();

  std::thread thr = std::thread([this]()
  {
    ExecResultMsg msg = this->grav_comp->recPredefPoses();
    recPredefPosesAckSignal(msg);
  });
  thr_::makeThreadRT(thr);
  thr.detach();
}

void MainWindow::recPredefPosesAckSlot(const ExecResultMsg &msg)
{
  rec_predef_poses = false;
  showMsg(msg);
  updateGUIonRecPredefPoses();
}

void MainWindow::updateGUIonRecPredefPoses()
{
  bool set = !rec_predef_poses();

  calc_CoM_btn->setEnabled(set);
  clear_wrenchQuat_btn->setEnabled(set);
  rec_wrenchQuat_btn->setEnabled(set);
  rec_predef_poses_btn->setEnabled(set);
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
  calc_CoM = false;
  clear_WrenchQuat_data = false;
  rec_wrenchQuat = false;
  rec_predef_poses = false;
  load_data = false;
  save_data = false;

  std::thread([this](){ this->grav_comp->setMode(rw_::IDLE); }).detach();

  // update_gui_sem.notify(); // unlock possible waits...
  QMainWindow::closeEvent(event);
}

void MainWindow::robotNotOkSlot(const QString &msg)
{
  rec_predef_poses = false;

  QMessageBox msg_box;
  msg_box.setText(msg + "\nDo you want to close the program?");
  msg_box.setIcon(QMessageBox::Critical);
  msg_box.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
  msg_box.setModal(true);

  int res = msg_box.exec();

  if (res == QMessageBox::Yes) this->close();
  // else do nothing ...
}
