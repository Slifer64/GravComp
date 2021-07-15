#include <gui_lib/label_lineEdit.h>

#include <iostream>

namespace as64_
{

namespace gui_
{

LabelLineEdit::LabelLineEdit(const std::string &label, const std::string &value, QWidget *parent): QWidget(parent)
{
  QFont font = QFont("Ubuntu", 14, 57);

  lb = new QLabel(label.c_str());
  lb->setFont(font);
  le = new QLineEdit();
  le->setText(value.c_str());
  le->setAlignment(Qt::AlignCenter);
  le->setFont(font);

  QHBoxLayout *layout = new QHBoxLayout(this);
  layout->addStretch(0);
  layout->addWidget(lb);
  layout->addWidget(le);

  QObject::connect( le, &QLineEdit::editingFinished, this, [this](){ emit this->editingFinished(); });
}

LabelLineEdit::~LabelLineEdit()
{

}

void LabelLineEdit::setFont(const QFont &font)
{
  lb->setFont(font);
  le->setFont(font);
}

void LabelLineEdit::setLabelFont(const QFont &font)
{
  lb->setFont(font);
}

void LabelLineEdit::setValueFont(const QFont &font)
{
  le->setFont(font);
}

} // namespace gui_

} // namespace as64_
