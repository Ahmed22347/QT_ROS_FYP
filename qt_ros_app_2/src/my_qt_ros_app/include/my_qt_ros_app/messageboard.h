#ifndef MESSAGEBOARD_H
#define MESSAGEBOARD_H

#include <QWidget>
#include <QTextEdit>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QString>
#include <QComboBox>
#include "mailbox.h"


class MessageBoard : public QWidget
{
    Q_OBJECT
private:
    QString color;

public:
    MessageBoard(const QString &sender, const QString &color,Mailbox* mailbox, Mailbox* other_mailbox, QWidget *parent = nullptr);
    void setInputWidget(QWidget *widget);
    void hideInputWidget();  // Add this method to hide the input widget
    void showInputWidget();
    void editInputWidget(int inputmessage);
    QString returnColour();
    void Restore_home();


    int *major_space;
    int *minor_space;
    int *major_turns;
    int *minor_turns;
    int *punishment_type;
    int *punishment_turns;

    int *bluereportbuttonadded;
    int *greenreportbuttonadded;

    int *message_active_blue;
    int *message_active_green;

    int *blue_num_of_favours;
    int *green_num_of_favours;

    int *bluecompelsion;
    int *greencompelsion;

    int getfavours(const QString& message);
    //void ThreadMessage(QString message);

    int favours=0;
signals:
    //void messageSent(const QString &sender, const QString &message);
    void relinquishaccess(const QString item);
    void messagereceived(const QString &sender, const QString &message);


public slots:
    void receiveMessage(const QString sender, const QString message);
    void playerMoved();
    void add_ReportButton(const QString &cartoinform);
    void move_message(const QString &message);
    void toggle_widget();


private slots:
    void sendMessage();

private:
    Mailbox *my_mailbox;
    Mailbox *other_mailbox;
    QTextEdit *messageArea;
    QWidget *inputWidget;
    QLineEdit *inputField;
    QString sender;
    int inputresponse(const QString &message);
    int toggle=0;
    QStringList generateCombinations(int removeMajorDominant, int majorNumberToRemove, int removeMinorDominant, int minorNumberToRemove);
    QStringList generateCompulsionTimeoutCombinations(int timeout, int turns);


    //QString color;

    void appendMessage(const QString &sender, const QString &message, bool isSender);

};

#endif // MESSAGEBOARD_H
