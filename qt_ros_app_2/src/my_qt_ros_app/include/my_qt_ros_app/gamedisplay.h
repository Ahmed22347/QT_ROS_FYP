#ifndef GAMEDISPLAY_H
#define GAMEDISPLAY_H

#include <QWidget>
#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QString>
#include <QFont>
#include <QDebug>

class GameDisplay : public QWidget {
    Q_OBJECT

public:
    //GameDisplay(QWidget *parent = nullptr);
    GameDisplay(int *major_space = nullptr, int *minor_space = nullptr, int *punishment_type = nullptr, int *punishment_turns = nullptr, int *major_turns = nullptr, int *minor_turns = nullptr, QWidget *parent = nullptr);

public slots:
    void updateLabelValue(int index, int value);
    void updateContainerB();
    void updateFirstContainer(int outstandingFavours, int favoursToCallIn, int compulsion, int timeout);
    void updateSecondContainer(int outstandingFavours, int favoursToCallIn, int compulsion, int timeout);
    void updatedisplay();



public:
    void updateMainSpaceLabelColor(const QString &color);
    void updateMainSpaceLabelContent(int value);
    void clearMainSpaceLabel();
    void updateMinorSpaceLabelColor(int index, const QString &color);
    void updateMinorSpaceLabelContent(int index, int value);
    void clearMinorSpaceLabel(int index);

    QWidget *topWidget;
    QWidget *bottomWidget;
    QWidget *firstContainer;
    QWidget *secondContainer;
    QWidget *spacesContainer;


    QWidget *containerA;
    QWidget *containerB;


    QLabel *mainSpaceLabel;
    QLabel *minorSpaceLabels[4];

    int *major_space;
    int *minor_space;
    int *punishment_type;
    int *punishment_turns;
    int *major_turns;
    int *minor_turns;

    QString *main_space_player;
    QString (*minor_space_player)[2][2];
    int *current_number_of_turns;
    int (*minor_number_of_turns)[2][2];

private:
    void setupLabel(QLabel* label);

    QList<QLabel*> topLabels;

    QHBoxLayout *layoutB;

    QLabel *label0;
    QLabel *label1;
    QLabel *label2;
    QLabel *label2_5;
    QLabel *label3;
    QLabel *label4;
    QLabel *label5;
    QLabel *label6;

    QLabel *widget0;
    QLabel *widget1;
    QLabel *widget2;
    QLabel *widget3;
    QLabel *widget4;




};

#endif // GAMEDISPLAY_H
