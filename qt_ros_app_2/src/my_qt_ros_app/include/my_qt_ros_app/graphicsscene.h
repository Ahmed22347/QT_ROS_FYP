#ifndef GRAPHICSSCENE_H
#define GRAPHICSSCENE_H

#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <QGraphicsPixmapItem>
#include <QColor>
#include <QPixmap>
#include <QPainter>
#include <QDebug>
#include <vector>
#include <QParallelAnimationGroup>
#include <QPropertyAnimation>
#include <QParallelAnimationGroup>
#include <array>
#include "twoplayerboard.h"
#include <QGraphicsSceneMouseEvent>
#include "custompixmapitem.h"
#include <QMutex>
#include <QMutexLocker>
#include <QGraphicsScene>
#include <QElapsedTimer>
#include "robotmanipulator.h"

class GraphicsScene : public QGraphicsScene {
Q_OBJECT

protected:
    void drawBackground(QPainter *painter, const QRectF &rect) override;
    void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
private:
    int gridSize;
    const int squareSize;
    //const std::array<std::array<Piece, 8>, 8>& grid;

    void startAnimation(CustomPixmapItem& pixmap);

     bool simulationMode_;	
    QString& getElementFromArray(int i, int j);
    int& getElementFromturns(int i, int j);
    void compareAndReplace(QString item, int skipI, int skipJ);
    void compareAndDoubleReplace(QString item, int skipI, int skipJ,  int skipI1, int skipJ1);

    std::vector<QString> returnAffectedSpaces(Step step, PieceType car_type);
    void compareandReplaceString(std::vector<QString> shared_vector, QString requesting_arm);

public:
    GraphicsScene(int gridSize, int squareSize,bool simulationMode_ ,QObject *parent = nullptr);
    static void removevectorItem(std::vector<std::string>& vec, const std::string& item);
    void addImageItem(int row, int col, const QString &imagePath, Orientation orientation, float scale);

    void updateGrid_cars(const std::array<std::array<Piece, 8>, 8> &grid);
    void removeImageItem(int row, int col);
    void Addboard(TwoPlayerBoard *board_to_add);
    //void compareAndReplaceAll(QString item);

    int ifmessage_active(Step movestep);
    void animateTranslation(int startRow, int startCol, int endRow, int endCol, int duration = 18000);
    void animateRotation(int row, int col, qreal startAngle, qreal endAngle, int duration = 18000);
    void animateBusTranslation(int startRow, int startCol, int endRow, int endCol, PieceType pieceType, Orientation pieceOrientation, int duration = 18000);
    void updatesharedspaces(const QString &input);
    int alternate =1;

    QGraphicsRectItem *selectedSquare;
    CustomPixmapItem *selectedImage;
    QMap<QGraphicsItem*, QGraphicsItem*> animationMap; // Map to keep track of animations
    std::map<CustomPixmapItem*, std::string> pixMap;
    bool *bluebutton;
    bool *greenbutton;
    int* blue_message_received;
    int* green_message_received;

    void remove_fail(QString color);
    QString thread_move_message(QString& car_colour, Step& step);
    int thread_movecar(Step step, QString requesting_arm);
    int move_intermediate_car_bus(Step step, QString vehicle, QString requesting_arm, PieceType pieceType=PieceType::NONE);
    QMutex mutex;
    QMutex thread_arms_mutex;
    QMutex movemessage_mutex;
    QMutex compelsionmutex;
    QMutex waiting_mutex;
    QString *main_space_player;
    QString (*minor_space_player)[2][2];

    int *current_number_of_turns;
    int (*minor_number_of_turns)[2][2];
    int *main_shared_space;// turns or dominant
    int *minor_shared_space; // turns or dominant
    int *num_main_space; //max turns
    int *num_minor_space; //max turns

    int blue_steps=0;
    int green_steps=0;

    int *bluetimeout;//blue should observe timeout
    int *greentimeout;
    QElapsedTimer *elapsedtime;
    float *winnertime;
    float *endtime;
    int *bluereportbuttonadded;
    int *greenreportbuttonadded;

    int* bluecompelsion;//number of complels that blue can do
    int* greencompelsion;

    int blue_activate_compulsion =0; //active when blue is compelled
    int green_activate_compulsion= 0;
    int *message_active_blue;
    int *message_active_green;
    Step check_and_return_compulsion(Step step);
    Step recreateMoveStepFromMessage(const QString& message);
    std::vector<std::string> arms;
    std::vector<std::string> thread_arms;
    QString read_main_splace_player();

    std::vector<QString> waiting_main_space_player;
    std::vector<QString> waiting_minor_space_player[2][2];
    TwoPlayerBoard *board;

    int punishement; // turns or dominant

   std::vector<Step> blue_message_stored;
   std::vector<Step> green_message_stored;

   void checkifstep(Step step);//reduce favour
   int *blue_num_of_favours;
   int *green_num_of_favours;
   bool *winner;
   int *endgame;
   float *time_blue;
   float *time_green;
   
   void add_collision_object();
   
   RobotManipulator *manipulator;
   RobotManipulator *manipulator2;
   ros::NodeHandle* nodeHandle_= nullptr;
   ros::NodeHandle* nodeHandle_2= nullptr;

signals:
    void sendshareSignal();
    void replytomessage();
    void replytomessageb();
    void opponentscar_being_used(const QString &cartoinform);
    void updatetimeouts();
    void movetobemade(const QString &move);
    void movetobemadeb(const QString &move);
    void togglewidgeta();
    void togglewidgetb();
    void show_endscreen_signal();
    void triggerPickAndPlace(const Step& step);
    void triggerPickAndRotate(const Step& step);

public slots:
    void compareAndReplaceAll(QString item);
    void thread_animation(Step step, QString arm_colour);

private slots:
    void handleAnimationFinished(QGraphicsItem *originalItem);
    void handleRotationFinished(QGraphicsItem *originalItem);
    void handleBusAnimationFinished(QList<QGraphicsRectItem*> busParts, int startRow, int startCol, int endRow1, int endRow2,int endRow3, int endCol);
    void stopAnimation(CustomPixmapItem* pixmap);

private:
    void checkmessage();
    QVector<QVector<QGraphicsRectItem*>> squares;
    bool checkAnimation();
    void initializeScene();
    QColor getColor(const QString &colorCode);
    bool checkCoordinates(int startRow, int startCol, int endRow, int endCol) const;
    bool checkCoordinates_message (int startRow, int startCol, int endRow, int endCol, int color) const;    //blue is 0 green =1
    bool checkStartCoordinate(int startRow, int startCol) const;
    int green_started;
    int blue_started;
    int board_num_of_turns;
    int checktimeout(QString arm);
    void checkpoint_translation(Step& new_step, int& startRow,int& startCol,int& endRow,int& endCol, QString& robot_arm);
    void checkpoint_rotation(Step& new_step, QString& robot_arm,  PieceType& pieceType, Orientation& pieceorientation, int &row, int &col);

    void check_ifopponent_car(Position carposition, QString armcolour, Step step);
    
        void applyKernelAtPosition(std::array<std::array<QString, GRID_SIZE>, GRID_SIZE>& grid, int i, int j, const QString& mark);
    void applyKernel(std::array<std::array<QString, GRID_SIZE>, GRID_SIZE>& grid, const Step& step, const QString& mark);
    void removeKernel(std::array<std::array<QString, GRID_SIZE>, GRID_SIZE>& grid, const QString& mark);

    bool checkStepPositions(const Step& step);
    

    const QString colorMatrix[8][8] = {
        {"B", "B", "LB", "BL", "BL", "LB", "LB", "LB"},
        {"B", "B", "GB", "BL", "BL", "GB", "LB", "LB"},
        {"LB", "LB", "GB", "Gr", "Gr", "GB", "LB", "LB"},
        {"LB", "LB", "Gr", "Gr", "Gr", "Gr", "LB", "LB"},
        {"LG", "LG", "Gr", "Gr", "Gr", "Gr", "LG", "LG"},
        {"LG", "LG", "GG", "Gr", "Gr", "GG", "LG", "LG"},
        {"LG", "LG", "GG", "BL", "BL", "GG", "G", "G"},
        {"LG", "LG", "LG", "BL", "BL", "LG", "G", "G"}
    };

    const QString colorboundaries[8][8] = {
        {"B", "B", "B", "L", "L", "B", "B", "B"},
        {"B", "B", "B", "L", "L", "B", "B", "B"},
        {"B", "B", "B", "A", "A", "B", "B", "B"},
        {"B", "B", "A", "A", "A", "A", "B", "B"},
        {"G", "G", "A", "A", "A", "A", "G", "G"},
        {"G", "G", "G", "A", "A", "G", "G", "G"},
        {"G", "G", "G", "L", "L", "G", "G", "G"},
        {"G", "G", "G", "L", "L", "G", "G", "G"}
    };
    
std::array<std::array<QString, GRID_SIZE>, GRID_SIZE> time_delay_grid = {{
    {"N", "N", "N", "N", "N", "N", "N", "N"},
    {"N", "N", "N", "N", "N", "N", "N", "N"},
    {"N", "N", "N", "N", "N", "N", "N", "N"},
    {"N", "N", "N", "N", "N", "N", "N", "N"},
    {"N", "N", "N", "N", "N", "N", "N", "N"},
    {"N", "N", "N", "N", "N", "N", "N", "N"},
    {"N", "N", "N", "N", "N", "N", "N", "N"},
    {"N", "N", "N", "N", "N", "N", "N", "N"}
}};


};

#endif // GRAPHICSSCENE_H
