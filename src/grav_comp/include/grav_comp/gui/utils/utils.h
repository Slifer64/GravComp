#ifndef GUI_UTILS_H
#define GUI_UTILS_H

#include <QString>
#include <QMessageBox>
#include <QLineEdit>
#include <QPushButton>

#include <mutex>
#include <condition_variable>

#include <grav_comp/utils.h>

int showErrorMsg(const QString &msg);
int showWarningMsg(const QString &msg);
int showQuestionMsg(const QString &msg);
int showInfoMsg(const QString &msg);
int showMsg(const ExecResultMsg &msg);

class MyLineEdit: public QLineEdit
{
public:
    MyLineEdit(QWidget *parent=0):QLineEdit(parent) { size_hint = QSize(60,30); }

    void setSizeHint(int w, int h) { size_hint.setWidth(w); size_hint.setHeight(h); }
    QSize sizeHint() const override { return size_hint; }
private:
    QSize size_hint;
};

#endif // GUI_UTILS_H
