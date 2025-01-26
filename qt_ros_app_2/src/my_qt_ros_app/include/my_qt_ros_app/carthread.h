#ifndef CARTHREAD_H
#define CARTHREAD_H

#include <array>
#include <iostream>
#include <random>
#include <thread>
#include <vector>
#include <cmath>
#include <limits>
#include <set>
#include <algorithm>
#include <fstream>
#include <string>
#include <queue>
#include "twoplayerboard.h"
#include <QThread>
#include <QMutex>
#include <QString>
#include "graphicsscene.h"  // Assuming you have these classes defined elsewhere
#include "messageboard.h"
#include "mailbox.h"
#include <QDebug>
using namespace std;

class CarThread : public QThread
{
    Q_OBJECT

public:
    enum Phase { Planning, Continuous };
   std::atomic<bool> stopFlag;

    CarThread(const QString &car_color,const QString firstboardcolor, QMutex *mutex, int *messageCounter, int *firstSpace,bool* gamestarted,
              GraphicsScene &scene, MessageBoard &messageentry, Mailbox *mymessageboard, Mailbox *othermessageboard,
              int &Endgame, QObject *parent = nullptr);
    ~CarThread();


//    void stop() {
//        stopFlag.store(true);  // Signal the thread to stop
//    }

    void stop();
    bool terminate=false;
    int* current_share_space_arrangemant;
    int* current_shared_space_number;
    int* minor_shared_space_arrangement;
    int (*minor_shared_space_number) [2][2];
    int* max_shared_space_number;
    int* max_minor_shared_space_number;
    bool* gamestarted;
    QMutex *mutex;
   // QMutex *thread_arms_mutex;
    int *messageCounter;
    int *firstSpace;  //probaly not used
    int *first_space_first_phase;
    int *penalty_type;
    int *mycompelsion=0;//number of complels that blue can do
    int *opponentcompelsion=0;
    int *myTimeout=0; //timeout blue should serve
    int *opponentTimeout=0;// timeout green should serve
    int *my_num_of_favours;
    int *opponenent_num_of_favours;
    int *numbe_of_punishment_turns;
    int *bluereportbuttonadded = nullptr;
    int *greenreportbuttonadded = nullptr;
    QString *main_space_player ;
    QString (*minor_space_player) [2][2];
    bool running;
    Position prev_step_cuurrnt_position = Position(0,0);
    Position prev_step_next_position = Position(0,0);
    MOVEMENT prev_movement = MOVEMENT::ROTATION;
    int _num_extra=0;
   // void setMutex(QMutex *mutex);

signals:
    void thread_animation(Step step, QString arm_colour);

protected:
    void run() override;

private:
    struct Node {
        Position pos;
        int gCost;
        float hCost;
        float fCost() const { return gCost + hCost; }
        Node* parent; // Used to reconstruct the path

        Node(Position position = Position(), Node* pParent = nullptr) : pos(position), gCost(0), hCost(0), parent(pParent) {}

        bool operator==(const Node& other) const {
            return pos.i == other.pos.i && pos.j == other.pos.j;
        }
        bool operator<(const Node& other) const {
            return this->fCost() < other.fCost();
        }
    };

    std::string orientationToString(Orientation orientation);

    float affinity_multiplier_1= 0.1f; //additive
    float affinity_multiplier_change=1; //multiplied by additive
    float affinity_promise= 0.1f; //hope of buliding affinty
    float affinity = 0.5;
    float failure_rate;
    int messagecost=2;
    int messagecost_increase=0;
    int working_meesage_cost = 0;
    Step sentmesssage;
    float lambda_main_space=0.2f;
    int risk_int = 0;  //risk -5 to 5 a high risk paerson, will be willing to make risky decisions // like cheating
    int disagreabbleness =2;//in ascale of -5 to 5 , can be devide by 10 to turn into float, 10 highly disabgreable.
    int risk_of_second_turn=3;
    float favour_multiplier=1.0; //variable reduces when favour is not replied range 0-2
    std::vector<std::vector<ActionItem>> fastestactionpath;
    //std::vector<std::vector<ActionItem>> bestActionsVector;
    int actionsvector_not_empty;


    ///initialise when needed
    int major_dissatisfaction=0;
    float minor_dissatisfaction=0;

    int want_main_shared_space_number =0;
    int want_minor_shared_space_number=0;
    GraphicsScene &scene;
    MessageBoard &messageentry;
    Mailbox *messageboard;
    Mailbox *othermessageboard;
    int &Endgame;
    QString car_color;
    PieceType car_type;
    const QString firstboardcolor;

    bool firstboard = false;
    bool phase1 = false;  // This seems to be a flag that may be used to indicate a phase
    int message = -1;     // Message variable placeholder
    int action_message=0;//0 do action 1 tell opponent to do it

    int sleep_time= 1000;//for thread
    void thread_runner();
    void propose_main_shared_space_assignment();
    void propose_minor_shared_space_assignment();
    void propose_penalties();
    int calculate_want_space_number(int main_or_minor);
    int calculate_wanted_penalties();
    std::vector<std::vector<ActionItem>> getfastestsvectors();
    float want_penalties_number;
    std::vector<int> counters;
    int closestpawn_number = 1;
        
    QString longestSequenceWord;    //major
    int longestGSequenceOverall = 0;
    int totalGInLongestSequences = 0;
    int averageConsecutiveGs = 0;        
    int firstaccesswant_main=0;
    int firstaccesswant_minor= 0;
    int totalLongestSequence_minor = 0;
    QString longestStartingSequenceWord_minor; //minor
    double maxAverageSequence_minor = 0.000000; //calculates the average sequence length for each character (a, b, c, d) across all words, then finds the maximum of these averages.
    double averageLongestChainSequence_minor = 0.0000000; //the average of the longest chain sequences across all characters. This is the mean value of the sum of longest sequences found for each character.
    float lambda_minor_space= 0.70000f;
    int failure_count  =0;
//f
    //messaging
    QString major_result;
    bool major_result_bool=false;
//    QString minor_result;
//    bool minor_result_bool;
        
    // Function declarations
    float EuclidianDistance(const Position&start,const Position&end);
    bool isValidPosition(const Position&pos,int car_colour);
    bool isValidBusPosition(const Position&pos,int car_colour);
    std::vector<Position> getNeighbors(const Position&pos,int car_colour);
    std::vector<Position> getBusNeighbors(const Position&pos,int car_colour,PieceType busType,Orientation busOrientation);
    std::vector<Position> reconstructPath(Node*currentNode);
    std::vector<Position> findPathAStar(const std::array<std::array<Piece,GRID_SIZE>,GRID_SIZE>grid,Position start,Position end);
    Position findClosestPawn(const std::array<std::array<Piece,GRID_SIZE>,GRID_SIZE>&grid,PieceType coloredCar,int number);
    Position find_end(PieceType coloredCar,const std::array<std::array<Piece,8>,8>&grid);
    void printPath(const std::vector<Position>&path);
    std::vector<Position> findNextStep(Position closest_pawn,const std::array<std::array<Piece,8>,8>&grid,PieceType coloredCar);
    int writesteptologfile(std::string textfile,Step step);
    int writemessagetologfile(std::string textfile,Position position);
    Step returnStep(Orientation moveorientation,Orientation car_orientation,Position path_0,Position path_1,PieceType& colored_car);
    std::vector<Position> getblockingcarsposition(const std::array<std::array<Piece,8>,8>&grid,std::vector<Position>& path);
    int check_existing(const std::array<std::array<Piece,8>,8>&grid,Position viable_neighbour,std::vector<Position>& added_cars);
    int Removeblocking_car(Position blocking_car_position,TwoPlayerBoard&board,int&movecounter,std::vector<Position>& cars_to_be_moved, int car_colour, std::vector<ActionItem>& actionItems);
    int Advance(std::vector<Position>&path,Position closest_pawn,PieceType&colored_car,TwoPlayerBoard board, std::vector<ActionItem>& actionItems);
    int get_number(const std::array<std::array<Piece, 8>, 8>& grid);
    int IsPawninFinal_destination();
    int Areallpawnsin_Final_destination(const std::array<std::array<Piece, 8>, 8>& grid);
    int handleguidingothercar(Position blocking_car_position, int& car_colour, TwoPlayerBoard& board, int& movecounter, std::vector<Position>& cars_to_be_moved, std::vector<ActionItem>& actionItems);
    QString determineLetter(const Position& current, const Position& next, const QString (&colorMatrix)[8][8], int& main_minor_number) ;
    std::vector<QString> processFastestVectors(const std::vector<std::vector<ActionItem>>& fastestvectors, const QString (&colorMatrix)[8][8], int& main_minor_number);
    int findLongestGSequence(const QString& word);
    int countGInLongestSequences(const QVector<QString>& words, const QString& longestSequenceWord, int longestGSequence);
    void analyzeGSequences(const std::vector<QString>& words, int& longestGSequenceOverall, int& totalGInLongestSequences, int& averageConsecutiveGs);
    int findLongestSequence_minor(const QString& word, QChar target);
    int findLongestStartingSequence_minor(const QString& word, QChar target);
    void analyzeAndMaxAverageSequences(const QVector<QString>& words, int& totalLongestSequence, QString& longestStartingSequenceWord, double& maxAverageSequence, double& averageLongestChainSequence);
    void handlemessage();
    void handlesatisfactionlevels();
    int compare_step_with_resources(QString& owned_ressources, QString& needed_ressources);
    QString getwords_owned();
    QString getwords_needed(Step& step, const QString (&colorMatrix)[8][8]);
    std::vector<std::vector<ActionItem>> getSortedActionItems(const std::map<int, std::pair<int, std::vector<ActionItem>>>& myMap);
    QString getSpaceStateFavour(double number,int major_or_minor);
    bool containsQString(QString qstr, std::vector<std::string> vec);
    QVector<int> findAllSequences_minor(const QString& word, QChar target);
   QString subtractStrings(const QString& str1, const QString& str2);
   bool shouldExclude(const Position& pos, const Position& start, const Position& goal);

};

#endif // CARTHREAD_H
