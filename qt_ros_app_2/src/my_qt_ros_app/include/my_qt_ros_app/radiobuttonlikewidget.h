#ifndef RADIOBUTTONLIKEWIDGET_H
#define RADIOBUTTONLIKEWIDGET_H

#include <QWidget>
#include <QPushButton>

class RadioButtonLikeWidget : public QWidget {
    Q_OBJECT

public:
    bool button1Selected;
    bool button2Selected;

    RadioButtonLikeWidget(QWidget *parent = nullptr);

    // Method to check which button is selected
    bool isButton1Selected() const;
    bool isButton2Selected() const;


private slots:
    void onButton1Clicked();
    void onButton2Clicked();

private:
    QPushButton *button1;
    QPushButton *button2;


    void updateButtonStyles();
};

#endif // RADIOBUTTONLIKEWIDGET_H
