#ifndef GUI_LIB_LABEL_LINEEDIT_H
#define GUI_LIB_LABEL_LINEEDIT_H

#include <QLabel>
#include <QLineEdit>
#include <QHBoxLayout>

namespace as64_
{

namespace gui_
{

class LabelLineEdit : public QWidget
{
Q_OBJECT

public:
  LabelLineEdit(const std::string &label, const std::string &value, QWidget *parent = 0);
  ~LabelLineEdit();

  double getValue() const { return le->text().toDouble(); }
  void setValue(double val) { le->setText(QString::number(val)); }

  void setFont(const QFont &font);

  void setLabelFont(const QFont &font);
  void setValueFont(const QFont &font);

signals:
    void editingFinished();

private:

  QLabel *lb;
  QLineEdit *le;
};

} // namespace gui_

} // namespace as64_

#endif // GUI_LIB_LABEL_LINEEDIT_H
