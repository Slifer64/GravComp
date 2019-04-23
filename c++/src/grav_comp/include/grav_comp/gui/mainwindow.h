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
#include <grav_comp/utils.h>
#include <grav_comp/robot/robot.h>

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    enum Mode
    {
      RUN_CONTROLLER = 11,
      FREEDRIVE = 31,
      IDLE = 57
    };

    MainWindow(const Robot *robot, QWidget *parent = 0);
    ~MainWindow();

    void setMode(const Mode &m);
    MainWindow::Mode getMode() const;
    QString getModeName() const;

    bool isRunning() const { return is_running; }

    bool calcCoM() const { return calc_CoM(); }
    void sendcalcCoMAck(bool success, const QString &msg)  { emit calcCoMAckSignal(success,msg); update_gui_sem.wait(); }

    bool recCurrentWrenchQuat() const { return rec_wrenchQuat(); }
    void sendRecCurrentWrenchQuatAck(bool success, const QString &msg)  { emit recWrenchQuatAckSignal(success,msg); update_gui_sem.wait(); }

    bool clearWrenchQuatData() const { return clear_WrenchQuat_data(); }
    void sendclearWrenchQuatDataAck(bool success, const QString &msg) { emit clearWrenchQuatDataAckSignal(success,msg); update_gui_sem.wait(); }

    bool loadData() const { return load_data(); }
    void sendLoadAck(bool success, const QString &msg) { emit loadAckSignal(success,msg); update_gui_sem.wait(); }
    std::string getLoadDataPath() const { return load_data_path; }

    bool saveData() const { return save_data(); }
    void sendSaveAck(bool success, const QString &msg) { emit saveAckSignal(success,msg); update_gui_sem.wait(); }
    std::string getSaveDataPath() const { return save_data_path; }

signals:
    void terminateAppSignal(const QString &msg);
    void controllerFinishedSignal(const QString &msg);
    void notTrainedSignal(const QString &msg);
    void modeChangedSignal();
    void loadAckSignal(bool success, const QString &msg);
    void saveAckSignal(bool success, const QString &msg);
    void calcCoMAckSignal(bool success, const QString &msg);
    void clearWrenchQuatDataAckSignal(bool success, const QString &msg);
    void recWrenchQuatAckSignal(bool success, const QString &msg);

private slots:
    void loadTriggered();
    void saveTriggered();
    void saveAsTriggered();
    void viewPoseTriggered();
    void viewJointsTriggered();
    void calcCoMPressed();
    void clearWrenchQuatDataPressed();
    void recWrenchQuatPressed();

    void loadAckSlot(bool success, const QString &msg);
    void saveAckSlot(bool success, const QString &msg);
    void startPoseAckSlot(bool success, const QString &msg);
    void clearWrenchQuatDataAckSlot(bool success, const QString &msg);
    void calcCoMAckSlot(bool success, const QString &msg);

    void notTrainedSlot(const QString &msg);
    void controllerFinishedSlot(const QString &msg);
    void terminateAppSlot(const QString &msg);
    void modeChangedSlot();

private:
    QWidget *central_widget;
    QStatusBar *status_bar;

    std::map<Mode, QString> mode_name;

    Semaphore update_gui_sem;

    Mode mode;

    bool is_running;

    MtxVar<bool> calc_CoM;
    MtxVar<bool> clear_WrenchQuat_data;
    MtxVar<bool> rec_wrenchQuat;
    MtxVar<bool> load_data;
    MtxVar<bool> save_data;
    std::string save_data_path;
    std::string default_data_path;
    std::string load_data_path;

    const Robot *robot;
    ViewPoseDialog *view_pose_dialog;
    ViewJPosDialog *view_jpos_dialog;

    // ======  menus  ========
    QMenuBar *menu_bar;
    QMenu *file_menu;
    QMenu *edit_menu;
    QMenu *view_menu;

    // ======  actions  ========

    QAction *load_CoM_act;
    QAction *save_act;
    QAction *save_as_act;

    QAction *view_pose_act;
    QAction *view_joints_act;

    // ======  widgets  ========
    QLabel *mode_label;
    QPushButton *run_ctrl_btn;
    QPushButton *freedrive_btn;
    QPushButton *idle_btn;

    QPushButton *load_CoM_btn;
    QPushButton *save_btn;
    QPushButton *save_as_btn;

    QPushButton *emergency_stop_btn;
    QPushButton *calc_CoM_btn;
    QPushButton *rec_wrenchQuat_btn;
    QPushButton *clear_wrenchQuat_btn;

    // ======  layouts  ========
    QVBoxLayout *mode_layout;
    QVBoxLayout *load_save_layout;
    QVBoxLayout *btns_layout;
    QGridLayout *main_layout;

    // ======  functions  ========
    void createWidgets();
    void createLayouts();
    void createConnections();
    void createActions();
    void createMenus();
    void updateInterfaceOnModeChanged();

    void updateInterfaceOnTrainPressed();
    void updateInterfaceOnGotoStartPose();
    void updateInterfaceOnLoadData();
    void updateInterfaceOnSaveData();

    void closeEvent(QCloseEvent *event) override;
};

#endif // MAINWINDOW_H
