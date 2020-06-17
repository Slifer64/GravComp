#ifndef GUI_BTN_WIDGET_H
#define GUI_BTN_WIDGET_H

#include <QPushButton>
#include <QAction>

#include <grav_comp/gui/utils/utils.h>
#include <grav_comp/robot/robot.h>

class MainWindow; // forward decleration

class BtnWidget : public QObject
{
  Q_OBJECT

public:

  BtnWidget(MainWindow *main_win,
               std::function<void()> on_trigger_fun,
               std::function<void(bool)> update_gui_fun,
               const std::string &btn_name, QFont btn_font);
  ~BtnWidget();

signals:
  void ackSignal(const ExecResultMsg &msg);

public slots:
  virtual void triggered();
  void ackSlot(const ExecResultMsg &msg);


protected:

  friend MainWindow;

  MtxVar<bool> enabled;
  QPushButton *btn;

  MainWindow *main_win;

  void updateGUI(bool set);
  void setEnabled(bool set);
  void setVisible(bool set);

  std::function<void()> on_trigger_fun;
  std::function<void(bool)> update_gui_fun;

};

#endif // GUI_BTN_WIDGET_H

