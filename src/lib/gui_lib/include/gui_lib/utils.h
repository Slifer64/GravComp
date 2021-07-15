#ifndef GUI_LIB_UTILS_H
#define GUI_LIB_UTILS_H

#include <memory>
#include <functional>

#include <QLineEdit>
#include <QApplication>
#include <QMainWindow>
#include <QThread>

namespace as64_
{

namespace gui_
{

class MyLineEdit: public QLineEdit
{
public:
    MyLineEdit(QWidget *parent=0):QLineEdit(parent) { size_hint = QSize(60,30); }

    void setSizeHint(int w, int h) { size_hint.setWidth(w); size_hint.setHeight(h); }
    QSize sizeHint() const override { return size_hint; }
private:
    QSize size_hint;
};

void launchGui(std::function<QMainWindow *()> create_mainwin_fun_, bool *gui_finished, int thr_priority=QThread::LowestPriority, std::function<void()>cookie_fun=nullptr);

} // namespace gui_

} // namespace as64_

#endif // GUI_LIB_UTILS_H
