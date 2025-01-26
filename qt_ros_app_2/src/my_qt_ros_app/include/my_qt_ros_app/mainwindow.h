#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "messageboard.h"
#include "twoplayerboard.h"
#include "radiobuttonlikewidget.h"
#include "graphicsscene.h"
#include "gamedisplay.h"
#include "mailbox.h"
#include <QComboBox>
#include <string>
#include <QGraphicsView>
#include <QGraphicsRectItem>
#include <QColor>
#include <QDebug>
#include "carthread.h"
#include <QCloseEvent>
#include <QElapsedTimer>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <QCheckBox>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

//    enum sharedSpace{
//        Turns,
//        Dominant
//    };

//    enum Punishment{
//        Timeout,
//        Compulsion
//    };

    QCheckBox* checkbox;
    bool simulation_mode;	
    int number_of_turns=10000000; //miorspace_latter_ rouns
    MessageBoard *startingboard;
    GameDisplay *gameDisplay;
    QGraphicsView *view;
     QElapsedTimer elapsedTimer;

    int bluereportbuttonadded=0;
    int greenreportbuttonadded=0;

    int bluecompelsion=0;//number of complels that blue can do
    int greencompelsion=0;
    int blueTimeout=0; //timeout blue should serve
    int greenTimeout=0;// timeout green should serve
    int num_of_favours=0;
    int numbe_of_punishment_turns= 10;
    int final_favour_numbers=0;
    MessageBoard *majorstartingboard;
    int punishment; //timeout 0 : compulsion 1
    bool *button_selected;
    int num_of_turns_majors_space =1000000;
    int blue_num_of_favours=0;
    int green_num_of_favours=0;
    
    ros::AsyncSpinner *spinner_;


    QString main_space_player = "-";
    QString minor_space_player[2][2]= {{"-", "-"},{"-", "-"}};
    int current_number_of_turns=10000000;
    int minor_number_of_turns[2][2] = {
        {1000000, 1000000},
        {1000000, 1000000}
    };

    bool winner;
    float time_winner;
    float end_time;
    float time_blue;
    float time_green;
    static int getfavours(const QString& message);
    static int returnnumber(int number, QString message);

private slots:
    void on_startButton_clicked();
    void on_messageSent(const QString &sender, const QString &message);
    void updatecounter(const QString &sender, const QString &message);
    void update_timeoutdisplay();
    void show_game_over_screen();

private:
    Ui::MainWindow *ui;
    QWidget *buttonWidget;
    TwoPlayerBoard *board;
    MessageBoard *messageBoard1;
    MessageBoard *messageBoard2;
    ///////////////Change afterRosmade1/////////////////////
    Mailbox* mailbox1;
    Mailbox* mailbox2;
    CarThread *carthread_blue;
    CarThread *carthread_green;
    ///////////////Change afterRosmade1/////////////////////
    RadioButtonLikeWidget *radiobutton;

    int messagecounter;
    bool isFirstMessenger;
    QString temporaryfirstmessage;
    int  majorspace;//minor_space in latter_rounds           Turns 0   Dominant 1
    int actual_majorspace;
    int findCharInString(const std::string& str);
    int temp_num_turns;
    int first_space;  //used to check if infirst loop to differentiate shared space
    int gridSize=8;
    const int squareSize = 160;
    MessageBoard *tempfirstaccess;
    GraphicsScene* scene;

    void setgame();
    void startgame();
    void setupfirstComboBoxForMessenger(MessageBoard *board);
    void setupYesNoComboBox(MessageBoard *board);
    void NoThankYouComboBox(MessageBoard *board);


    void updateValues(const QString &sender, const QString &message);

    int temporarymajorspace;
    int temporary_maj_space_num;
    int temporaryminorspace;
    int temporary_minor_space_num;
    int category = 0;
    int temporary_blue_favours = 0;
    int temporary_green_favours = 0;
    int temp_punishment=0;
    int temp_punishment_turns= 0;
        ///////////////Change after Rosmad/////////////////////
    QMutex threadmutex;
    int Endgame=0;
    bool gamestarted=false;
        ///////////////Change afterRosmad2/////////////////////
    int blue_message_received = 0;
    int green_message_received = 0;

    int message_active_blue=0;
    int message_active_green=0;
    void writeToCSV(float value1, float value2, const std::string& filename);

    void closeEvent(QCloseEvent *event) override;

};

#endif // MAINWINDOW_H
