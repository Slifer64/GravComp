#include <grav_comp/gui/btn_act_widget/btn_act_widget.h>
#include <grav_comp/gui/mainwindow/mainwindow.h>

BtnActWidget::BtnActWidget(MainWindow *main_win,
                          std::function<void(BtnActWidget*)> on_trigger_fun,
                          std::function<void(bool)> update_gui_fun,
                          const std::string &btn_name, QFont btn_font,
                          const std::string &act_name, const std::string &act_shcut, const std::string &act_info)
{
  this->main_win = main_win;
  this->on_trigger_fun = on_trigger_fun;
  this->update_gui_fun = update_gui_fun;

  this->enabled = false;

  btn = new QPushButton;
  btn->setText(btn_name.c_str());
  btn->setFont(btn_font);

  act = new QAction(tr(act_name.c_str()), this);
  if (!act_shcut.empty()) act->setShortcut(QKeySequence(act_shcut.c_str()));
  if (!act_info.empty()) act->setStatusTip(tr(act_info.c_str()));

  qRegisterMetaType<ExecResultMsg>("ExecResultMsg");

  QObject::connect( btn, &QPushButton::clicked, this, &BtnActWidget::triggered );
  QObject::connect( act, &QAction::triggered, this, &BtnActWidget::triggered );
  QObject::connect( this, &BtnActWidget::ackSignal, this, &BtnActWidget::ackSlot );

}

BtnActWidget::~BtnActWidget()
{

}

void BtnActWidget::triggered()
{
  on_trigger_fun(this);
}

void BtnActWidget::ackSlot(const ExecResultMsg &msg)
{
  enabled = false;
  updateGUI(true);
  showMsg(msg);
}

void BtnActWidget::setEnabled(bool set)
{
  act->setEnabled(set);
  btn->setEnabled(set);
}

void BtnActWidget::setVisible(bool set)
{
  act->setVisible(set);
  btn->setVisible(set);
}

void BtnActWidget::updateGUI(bool set)
{
   if (update_gui_fun) update_gui_fun(set);
}

// =======================================================================
// =======================================================================




