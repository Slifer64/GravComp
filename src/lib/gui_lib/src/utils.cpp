
#include <gui_lib/utils.h>

namespace as64_
{

namespace gui_
{

void launchGui(std::function<QMainWindow *()> create_mainwin_fun_, bool *gui_finished, int thr_priority, std::function<void()>cookie_fun)
{
  *gui_finished = false;

  int argc = 0;
  char **argv = 0;
  QApplication app(argc, argv);
  QThread::currentThread()->setPriority(static_cast<QThread::Priority>(thr_priority));

  QMainWindow *main_win = create_mainwin_fun_();

  if (cookie_fun) cookie_fun();

  main_win->show();
  app.exec();

  *gui_finished = true;
  delete main_win; // must be destructed in this thread!
}

} // namespace gui_

} // namespace as64_
