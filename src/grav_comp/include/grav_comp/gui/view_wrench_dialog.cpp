#include "view_wrench_dialog.h"

ViewWrenchDialog::ViewWrenchDialog(std::function<arma::vec()> readWrench, const std::string &win_name, QWidget *parent): QDialog(parent)
{
    run = false;

    this->readWrench = readWrench;

    if (!win_name.empty()) this->setWindowTitle(win_name.c_str());
    else this->setWindowTitle("Tool wrench");

    QLabel *pos_label = new QLabel("Force");
    pos_label->setStyleSheet("background-color: rgb(245,245,245); color: rgb(0,0,0); font: 75 14pt \"FreeSans\";");
    QLabel *x_label = new QLabel("x");
    QLabel *y_label = new QLabel("y");
    QLabel *z_label = new QLabel("z");
    QLabel *f_label = new QLabel("N");

    fx_le = createLineEdit();
    fy_le = createLineEdit();
    fz_le = createLineEdit();
    QLabel *t_label = new QLabel("Nm");

    QLabel *orient_label = new QLabel("Torque");
    orient_label->setStyleSheet("background-color: rgb(245,245,245); color: rgb(0,0,0); font: 75 14pt \"FreeSans\";");

    tx_le = createLineEdit();
    ty_le = createLineEdit();
    tz_le = createLineEdit();

    QGridLayout *main_layout = new QGridLayout(this);
    // main_layout->setSizeConstraint(QLayout::SetFixedSize);
    main_layout->addWidget(x_label,0,1, Qt::AlignCenter);
    main_layout->addWidget(y_label,0,2, Qt::AlignCenter);
    main_layout->addWidget(z_label,0,3, Qt::AlignCenter);
    main_layout->addWidget(pos_label,1,0, Qt::AlignCenter);
    main_layout->addWidget(fx_le,1,1);
    main_layout->addWidget(fy_le,1,2);
    main_layout->addWidget(fz_le,1,3);
    main_layout->addWidget(f_label,1,4);
    main_layout->addItem(new QSpacerItem(0,20),2,0);
    main_layout->addWidget(orient_label,3,0, Qt::AlignCenter);
    main_layout->addWidget(tx_le,3,1);
    main_layout->addWidget(ty_le,3,2);
    main_layout->addWidget(tz_le,3,3);
    main_layout->addWidget(t_label,3,4);

    Qt::ConnectionType connect_type = Qt::AutoConnection;
    MyLineEdit *le_array[] = {fx_le, fy_le, fz_le, tx_le, ty_le, tz_le};
    for (int i=0;i<6;i++) QObject::connect(le_array[i], SIGNAL(textChanged(QString)), le_array[i], SLOT(setText(QString)), connect_type);
}

ViewWrenchDialog::~ViewWrenchDialog()
{
    stop();
}

MyLineEdit *ViewWrenchDialog::createLineEdit()
{
    MyLineEdit *le = new MyLineEdit;
    le->setSizeHint(60,30);
    le->setReadOnly(true);
    le->setStyleSheet("font: 75 14pt;");
    le->setAlignment(Qt::AlignCenter);

    return le;
}

void ViewWrenchDialog::launch()
{
  if (!run)
  {
    run = true;
    std::thread(&ViewWrenchDialog::updateDialogThread, this).detach();
    this->show();
  }
}

void ViewWrenchDialog::stop()
{
  if (run)
  {
    run = false;
    this->hide();
  }
}

void ViewWrenchDialog::updateDialogThread()
{
  while (run)
  {
    arma::vec wrench = readWrench();

    emit fx_le->textChanged(QString::number(wrench(0),'f',2));
    emit fy_le->textChanged(QString::number(wrench(1),'f',2));
    emit fz_le->textChanged(QString::number(wrench(2),'f',2));

    emit tx_le->textChanged(QString::number(wrench(3),'f',2));
    emit ty_le->textChanged(QString::number(wrench(4),'f',2));
    emit tz_le->textChanged(QString::number(wrench(5),'f',2));

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

void ViewWrenchDialog::closeEvent(QCloseEvent *event)
{
    stop();
}
