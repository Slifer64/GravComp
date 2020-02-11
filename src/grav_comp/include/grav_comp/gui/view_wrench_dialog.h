#ifndef VIEW_WRENCH_DIALOG_H
#define VIEW_WRENCH_DIALOG_H

#include <QDialog>
#include <QLabel>
#include <QLineEdit>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QCloseEvent>

#include <functional>
#include <armadillo>
#include <thread>

#include "utils.h"

class ViewWrenchDialog : public QDialog
{
    Q_OBJECT

public:
    ViewWrenchDialog(std::function<arma::vec()> readWrench, const std::string &win_name="", QWidget *parent = 0);
    ~ViewWrenchDialog();

public slots:
    void launch();
    void stop();

private:
    MyLineEdit *fx_le;
    MyLineEdit *fy_le;
    MyLineEdit *fz_le;
    MyLineEdit *tx_le;
    MyLineEdit *ty_le;
    MyLineEdit *tz_le;

    bool run;
    std::function<arma::vec()> readWrench;

    void updateDialogThread();

    MyLineEdit *createLineEdit();

    void closeEvent(QCloseEvent *event) override;
};

#endif // VIEW_WRENCH_DIALOG_H
