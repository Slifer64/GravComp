#include <grav_comp/gui/btn_act_widget/act_widget.h>
#include <grav_comp/gui/mainwindow/mainwindow.h>

ActWidget::ActWidget(MainWindow *main_win,
                    std::function<void()> on_trigger_fun,
                    std::function<void(bool)> update_gui_fun,
                    const std::string &act_name, const std::string &act_shcut, const std::string &act_info)
{
  this->main_win = main_win;
  this->on_trigger_fun = on_trigger_fun;
  this->update_gui_fun = update_gui_fun;

  this->enabled = false;

  act = new QAction(tr(act_name.c_str()), this);
  if (!act_shcut.empty()) act->setShortcut(QKeySequence(act_shcut.c_str()));
  if (!act_info.empty()) act->setStatusTip(tr(act_info.c_str()));

  qRegisterMetaType<ExecResultMsg>("ExecResultMsg");

  QObject::connect( act, &QAction::triggered, this, &ActWidget::triggered );
  QObject::connect( this, &ActWidget::ackSignal, this, &ActWidget::ackSlot );

}

ActWidget::~ActWidget()
{

}

void ActWidget::triggered()
{
  on_trigger_fun();
}

void ActWidget::ackSlot(const ExecResultMsg &msg)
{
  enabled = false;
  updateGUI(true);
  showMsg(msg);
}

void ActWidget::setEnabled(bool set)
{
  act->setEnabled(set);
}

void ActWidget::setVisible(bool set)
{
  act->setVisible(set);
}

void ActWidget::updateGUI(bool set)
{
   if (update_gui_fun) update_gui_fun(set);
}

// =======================================================================
// =======================================================================




