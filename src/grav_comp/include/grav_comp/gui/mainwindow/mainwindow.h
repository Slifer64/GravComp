#ifndef GUI_MAINWINDOW_H
#define GUI_MAINWINDOW_H

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

#include <grav_comp/gui/view_pose_dialog/view_pose_dialog.h>
#include <grav_comp/gui/view_jpos_dialog/view_jpos_dialog.h>
#include <grav_comp/gui/view_wrench_dialog/view_wrench_dialog.h>
#include <grav_comp/gui/set_poses_dialog/set_poses_dialog.h>
#include <grav_comp/gui/btn_act_widget/act_widget.h>
#include <grav_comp/gui/btn_act_widget/btn_widget.h>
#include <grav_comp/utils.h>
#include <grav_comp/robot/robot.h>

class GravComp;

using namespace as64_;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    enum Mode
    {
      FREEDRIVE = 31,
      IDLE = 57
    };

    MainWindow(const Robot *robot, GravComp *grav_comp, QWidget *parent = 0);
    ~MainWindow();

    void setMode(const Mode &m);
    MainWindow::Mode getMode() const;
    QString getModeName() const;

    bool isRunning() const { return is_running; }

    bool recPredefPoses() const { return rec_predef_poses_->enabled(); }
    //void sendRecPredefPosesAck(const ExecResultMsg &msg)  { emit recPredefPosesAckSignal(success,msg); update_gui_sem.wait(); }

    std::string getLoadDataPath() const { return load_data_path; }
    std::string getSaveDataPath() const { return save_data_path; }

    std::vector<arma::vec> getPredefPoses() const { return set_poses_dialog->poses; }

signals:
    void robotNotOkSignal(const QString &msg);
    void modeChangedSignal();

private slots:
    void loadPredefPosesTriggered();
    void updateGUIonRecPredefPoses(bool set);

    void recPredefPosesPressed();

    void robotNotOkSlot(const QString &msg);
    void modeChangedSlot();

private:
    Mode mode; ///< The Robot control modes that can be setted through the GUI.
    std::map<Mode, QString> mode_name; ///< The names of Robot control modes that can be setted through the GUI.
    bool is_running; ///< Flag that signals if the GUI is running or not.
    // Semaphore update_gui_sem; ///< Semaphore for blocking/notifying an execution thread, so as to synchronize it with the GUI.

    ViewPoseDialog *view_pose_dialog; ///< Pointer to QDialog object for displaying the robot tool pose.
    ViewJPosDialog *view_jpos_dialog; ///< Pointer to QDialog object for displaying the robot's joint angles.
    ViewWrenchDialog *view_wrench_dialog; ///< Pointer to QDialog object for displaying the robot's tool wrench.
    ViewWrenchDialog *view_est_wrench_dialog; ///< Pointer to QDialog object for displaying the robot's tool wrench.
    SetPosesDialog *set_poses_dialog; ///< Pointer to QDialog object for setting predefined poses for recording wrench-orient.


    BtnWidget *calc_CoM_;
    void calcCoMPressed();
    void updateGUIonCalcCOMPressed(bool set);

    BtnWidget *rec_predef_poses_;

    BtnWidget *rec_wrenchQuat_;
    void recWrenchQuatPressed();
    void updateGUIonRecWrenchQuat(bool set);

    BtnWidget *clear_wrenchQuat_;
    void clearWrenchQuatDataPressed();
    void updateGUIonClearWrenchQuatData(bool set);

    ActWidget *load_CoM_;
    void loadCoMTriggered();
    void updateGUIonLoadCoMData(bool set);

    ActWidget *save_CoM_;
    ActWidget *save_as_CoM_;
    void saveCoMTriggered();
    void saveAsCoMTriggered();
    void updateGUIonSaveCoM(bool set);

    ActWidget *load_wrenchOrient_;
    void loadWrenchOrientTriggered();
    void updateGUIonLoadWrenchOrient(bool set);

    ActWidget *save_wrenchOrient_;
    void saveWrenchOrientTriggered();
    void updateGUIonSaveWrenchOrient(bool set);

    MtxVar<bool> load_wrenchOrient; ///< Flag that becomes true when loading of wrench-orient data is triggered and is reseted to false when the loading is done.
    MtxVar<bool> save_wrenchOrient; ///< Flag that becomes true when saving of wrench-orient data is triggered and is reseted to false when the saving is done.

    std::string default_data_path; ///< Default path were every load/save dialog that opens points to initially.
    std::string save_data_path; ///< Path to save the mass-CoM data.
    std::string load_data_path; ///< Path to load the mass-CoM data.

    GravComp *grav_comp; ///< Pointer to object with the execution functionalities.
    const Robot *robot; ///< Pointer to object with the robot functionality.

    // ======  menus  ========
    QMenuBar *menu_bar; ///< Menu Bar.
    QMenu *file_menu;  ///< File menu.
    QMenu *edit_menu; ///< Edit menu.
    QMenu *view_menu; ///< View Menu.

    // ======  actions  ========
    QAction *view_pose_act; ///< Triggers a QAction connected with the launch of 'view_pose_dialog'.
    QAction *view_joints_act; ///< Triggers a QAction connected with the launch of 'view_jpos_dialog'.
    QAction *view_wrench_act; ///< Triggers a QAction connected with the launch of 'view_wrench_dialog'.
    QAction *view_est_wrench_act; ///< Triggers a QAction connected with the launch of 'view_wrench_dialog'.
    QAction *set_predef_poses_act; ///< Triggers a QAction connected with the launch of 'set_predef_poses_dialog'.
    QAction *load_predef_poses_act; ///< Triggers a QAction connected with the loading of predefined poses.

    // ======  widgets  ========
    QWidget *central_widget;
    QStatusBar *status_bar;
    QLabel *mode_label; ///< QLabel displaying text.
    QPushButton *freedrive_btn; ///< QButton for setting the Robot mode to FREEDRIVE.
    QPushButton *idle_btn; ///< QButton for setting the Robot mode to IDLE.

    BtnWidget *emergency_stop_; ///< QButton for enabling emergency stop.
    void emergencyStopPressed();
    void updateGUIonEmergencyStop(bool set);

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

    /** Overrides the close event. */
    void closeEvent(QCloseEvent *event) override;
};

#endif // GUI_MAINWINDOW_H
