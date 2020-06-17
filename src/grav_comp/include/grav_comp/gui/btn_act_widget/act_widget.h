#ifndef GUI_ACT_WIDGET_H
#define GUI_ACT_WIDGET_H

#include <QPushButton>
#include <QAction>

#include <grav_comp/gui/utils/utils.h>
#include <grav_comp/robot/robot.h>

class MainWindow; // forward decleration

class ActWidget : public QObject
{
  Q_OBJECT

public:

  ActWidget(MainWindow *main_win,
               std::function<void()> on_trigger_fun,
               std::function<void(bool)> update_gui_fun,
               const std::string &act_name, const std::string &act_shcut="", const std::string &act_info="");
  ~ActWidget();

signals:
  void ackSignal(const ExecResultMsg &msg);

public slots:
  virtual void triggered();
  void ackSlot(const ExecResultMsg &msg);


protected:

  friend MainWindow;

  MtxVar<bool> enabled;
  QAction *act;

  MainWindow *main_win;

  void updateGUI(bool set);
  void setEnabled(bool set);
  void setVisible(bool set);

  std::function<void()> on_trigger_fun;
  std::function<void(bool)> update_gui_fun;

};

#endif // GUI_ACT_WIDGET_H

