#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMenuBar>
#include <QStatusBar>
#include <QToolBar>
#include <QMenu>
#include <QPushButton>
#include <QLabel>
#include <QAction>
#include <QWidget>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPalette>
#include <QColor>
#include <QMessageBox>
#include <QFileDialog>

#include <map>

#include "view_pose_dialog.h"
#include "view_jpos_dialog.h"
#include "view_wrench_dialog.h"
#include "set_poses_dialog.h"
#include <grav_comp/utils.h>
#include <robot_wrapper/robot.h>

class GravComp;

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:

  enum Mode
  {
    FREEDRIVE = 31,
    IDLE = 57
  };

  MainWindow(const rw_::Robot *robot, GravComp *grav_comp, QWidget *parent = 0);
  ~MainWindow();

  void setMode(const Mode &m);
  MainWindow::Mode getMode() const;
  QString getModeName() const;

  bool isRunning() const { return is_running; }

  bool recPredefPoses() const { return rec_predef_poses(); }
  //void sendRecPredefPosesAck(const ExecResultMsg &msg)  { emit recPredefPosesAckSignal(success,msg); update_gui_sem.wait(); }

  std::string getLoadDataPath() const { return load_data_path; }
  std::string getSaveDataPath() const { return save_data_path; }

  std::vector<arma::vec> getPredefPoses() const { return set_poses_dialog->poses; }

signals:
  void robotNotOkSignal(const QString &msg);
  void modeChangedSignal();
  void emergencyStopAckSignal();
  void loadAckSignal(const ExecResultMsg &msg);
  void saveAckSignal(const ExecResultMsg &msg);
  void loadWrenchOrientAckSignal(const ExecResultMsg &msg);
  void saveWrenchOrientAckSignal(const ExecResultMsg &msg);
  void calcCoMAckSignal(const ExecResultMsg &msg);
  void clearWrenchQuatDataAckSignal(const ExecResultMsg &msg);
  void recWrenchQuatAckSignal(const ExecResultMsg &msg);
  void recPredefPosesAckSignal(const ExecResultMsg &msg);

  void closeSignal();

private slots:
  void loadPredefPosesTriggered();
  void loadWrenchOrientTriggered();
  void saveWrenchOrientTriggered();
  void loadTriggered();
  void saveTriggered();
  void saveAsTriggered();
  void calcCoMPressed();
  void clearWrenchQuatDataPressed();
  void recWrenchQuatPressed();
  void recPredefPosesPressed();
  void emergencyStopPressed();

  void loadWrenchOrientAckSlot(const ExecResultMsg &msg);
  void saveWrenchOrientAckSlot(const ExecResultMsg &msg);
  void loadAckSlot(const ExecResultMsg &msg);
  void saveAckSlot(const ExecResultMsg &msg);
  void recWrenchQuatAckSlot(const ExecResultMsg &msg);
  void recPredefPosesAckSlot(const ExecResultMsg &msg);
  void clearWrenchQuatDataAckSlot(const ExecResultMsg &msg);
  void calcCoMAckSlot(const ExecResultMsg &msg);

  void robotNotOkSlot(const QString &msg);
  void modeChangedSlot();
  void emergencyStopAckSlot();

private:
  Mode mode; ///< The Robot control modes that can be setted through the GUI.
  std::map<Mode, QString> mode_name; ///< The names of Robot control modes that can be setted through the GUI.
  bool is_running; ///< Flag that signals if the GUI is running or not.
  // Semaphore update_gui_sem; ///< Semaphore for blocking/notifying an execution thread, so as to synchronize it with the GUI.

  thr_::MtxVar<bool> calc_CoM; ///< Flag that becomes true when the CoM calculation is triggered and is reseted to false when the calculation is done.
  thr_::MtxVar<bool> clear_WrenchQuat_data; ///< Flag that becomes true when clear of the recored data is triggered and is reseted to false when the clear is done.
  thr_::MtxVar<bool> rec_wrenchQuat; ///< Flag that becomes true when recording of data is triggered and is reseted to false when the recording is done.
  thr_::MtxVar<bool> rec_predef_poses; ///< Flag that becomes true when recording of data from predefined poses is triggered and is reseted to false when the recording is done.
  thr_::MtxVar<bool> load_data; ///< Flag that becomes true when loading of mass-CoM is triggered and is reseted to false when the loading is done.
  thr_::MtxVar<bool> save_data; ///< Flag that becomes true when saving of mass-CoM is triggered and is reseted to false when the saving is done.
  thr_::MtxVar<bool> load_wrenchOrient; ///< Flag that becomes true when loading of wrench-orient data is triggered and is reseted to false when the loading is done.
  thr_::MtxVar<bool> save_wrenchOrient; ///< Flag that becomes true when saving of wrench-orient data is triggered and is reseted to false when the saving is done.

  thr_::MtxVar<bool> emergency_stop;

  std::string default_data_path; ///< Default path were every load/save dialog that opens points to initially.
  std::string save_data_path; ///< Path to save the mass-CoM data.
  std::string load_data_path; ///< Path to load the mass-CoM data.

  GravComp *grav_comp; ///< Pointer to object with the execution functionalities.
  const rw_::Robot *robot; ///< Pointer to object with the robot functionality.
  ViewPoseDialog *view_pose_dialog; ///< Pointer to QDialog object for displaying the robot tool pose.
  ViewJPosDialog *view_jpos_dialog; ///< Pointer to QDialog object for displaying the robot's joint angles.
  ViewWrenchDialog *view_wrench_dialog; ///< Pointer to QDialog object for displaying the robot's tool wrench.
  ViewWrenchDialog *view_compWrench_dialog;
  SetPosesDialog *set_poses_dialog; ///< Pointer to QDialog object for setting predefined poses for recording wrench-orient.

  // ======  menus  ========
  QMenuBar *menu_bar; ///< Menu Bar.
  QMenu *file_menu;  ///< File menu.
  QMenu *edit_menu; ///< Edit menu.
  QMenu *view_menu; ///< View Menu.

  // ======  actions  ========
  QAction *load_predef_poses_act; ///< Triggers a QAction connected with the loading of predefined poses.
  QAction *load_wrenchOrient_act; ///< Triggers a QAction connected with the execution of the wrench-orient data load.
  QAction *save_wrenchOrient_act; ///< Triggers a QAction connected with the execution of the wrench-orient data save.
  QAction *load_CoM_act; ///< Triggers a QAction connected with the execution of the mass-CoM load.
  QAction *save_act; ///< Triggers a QAction connected with the execution of the mass-CoM save.
  QAction *save_as_act; ///< Triggers a QAction connected with the execution of the mass-CoM save as.
  QAction *view_pose_act; ///< Triggers a QAction connected with the launch of 'view_pose_dialog'.
  QAction *view_joints_act; ///< Triggers a QAction connected with the launch of 'view_jpos_dialog'.
  QAction *view_wrench_act; ///< Triggers a QAction connected with the launch of 'view_wrench_dialog'.
  QAction *view_compWrench_act;
  QAction *set_predef_poses_act; ///< Triggers a QAction connected with the launch of 'set_predef_poses_dialog'.
  QAction *view_ee_tf_act;
  QAction *view_CoM_tf_act;

  // ======  widgets  ========
  QWidget *central_widget;
  QStatusBar *status_bar;
  QLabel *mode_label; ///< QLabel displaying text.
  QPushButton *freedrive_btn; ///< QButton for setting the Robot mode to FREEDRIVE.
  QPushButton *idle_btn; ///< QButton for setting the Robot mode to IDLE.

  QPushButton *emergency_stop_btn; ///< QButton for enabling emergency stop.
  QPushButton *calc_CoM_btn; ///< QButton for triggering the execution of the mass-CoM calculation.
  QPushButton *rec_wrenchQuat_btn; ///< QButton for triggering the execution of wrench and quaternion recording.
  QPushButton *clear_wrenchQuat_btn; ///< QButton for triggering the execution of recorded wrench-quat data clearance.
  QPushButton *rec_predef_poses_btn; ///< QButton for triggering the execution of moving from predefined poses and recoring the wrench-quat at each pose.

  // ======  layouts  ========
  QVBoxLayout *mode_layout; ///< QLayout for storing the Robot mode buttons.
  QVBoxLayout *btns_layout;  ///< QLayout for storing the other buttons.
  QGridLayout *main_layout; ///< QLayout for storing all other layouts.

  // ======  functions  ========

  /** Creates all the widgets, i.e. buttons, labels etc. */
  void createWidgets();

  /** Creates the layouts and assigns the widgets to the corresponding layout. */
  void createLayouts();

  /** Sets up the connections between widget/action signals (i.e. clicked/triggered) and
  the corresponding functions (slots) that handle the signals. */
  void createConnections();

  /** Creates the QActions that will be stored in the menus. */
  void createActions();

  /** Creates the main menu and submenus in the GUI's menu bar and assigns
   * each action to the corresponding submenu. */
  void createMenus();

  /** Updates the interface when the Robot's mode changes. */
  void updateGUIonModeChanged();

  /** Updates the interface when the calculation of mass-CoM is triggered
   * ('calc_CoM'=true) and when it is finished ('calc_CoM'=false). */
  void updateGUIonCalcCOMPressed();
  void updateGUIonRecWrenchQuat();
  void updateGUIonRecPredefPoses();
  void updateGUIonClearWrenchQuatData();
  void updateGUIonLoadData();
  void updateGUIonSaveData();
  void updateGUIonLoadWrenchOrient();
  void updateGUIonSaveWrenchOrient();

  /** Overrides the close event. */
  void closeEvent(QCloseEvent *event) override;
};

#endif // MAINWINDOW_H
