#include <grav_comp/gui/btn_act_widget/btn_widget.h>
#include <grav_comp/gui/mainwindow/mainwindow.h>

BtnWidget::BtnWidget(MainWindow *main_win,
                      std::function<void()> on_trigger_fun,
                      std::function<void(bool)> update_gui_fun,
                      const std::string &btn_name, QFont btn_font)
{
  this->main_win = main_win;
  this->on_trigger_fun = on_trigger_fun;
  this->update_gui_fun = update_gui_fun;

  this->enabled = false;

  btn = new QPushButton;
  btn->setText(btn_name.c_str());
  btn->setFont(btn_font);

  qRegisterMetaType<ExecResultMsg>("ExecResultMsg");

  QObject::connect( btn, &QPushButton::clicked, this, &BtnWidget::triggered );
  QObject::connect( this, &BtnWidget::ackSignal, this, &BtnWidget::ackSlot );

}

BtnWidget::~BtnWidget()
{

}

void BtnWidget::triggered()
{
  on_trigger_fun();
}

void BtnWidget::ackSlot(const ExecResultMsg &msg)
{
  enabled = false;
  updateGUI(true);
  showMsg(msg);
}

void BtnWidget::setEnabled(bool set)
{
  btn->setEnabled(set);
}

void BtnWidget::setVisible(bool set)
{
  btn->setVisible(set);
}


void BtnWidget::updateGUI(bool set)
{
  if (update_gui_fun) update_gui_fun(set);
}

// =======================================================================
// =======================================================================




