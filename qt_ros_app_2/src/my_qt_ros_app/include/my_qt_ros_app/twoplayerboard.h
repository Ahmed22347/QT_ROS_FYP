#ifndef TWOPLAYERBOARD_H
#define TWOPLAYERBOARD_H


#pragma once

#include <array>
#include <iostream>
#include <random>
#include <tuple>
#include <QString>
#include <QMutex>
#include <QMutexLocker>
#include <QMetaType>


const int GRID_SIZE = 8;

// Enumeration for the orientation
enum class MOVEMENT {
    TRANSLATION, ROTATION
};

enum class Orientation {
    NORTH, EAST, SOUTH, WEST, NONE
};

struct Position {
    int i;
    int j;

    Position(int i = -1, int j = -1) : i(i), j(j) {}

    bool operator==(const Position& other) const {
        return i == other.i && j == other.j;
    }

    bool operator!=(const Position& other) const {
        return i != other.i || j == other.j;
    }

    bool operator<(const Position& other) const {
        // Compare the 'i' values first, and if they are equal, compare the 'j' values
        return std::tie(i, j) < std::tie(other.i, other.j);
    }
};

struct Step{
    MOVEMENT type = MOVEMENT::ROTATION;
    Orientation direction = Orientation::EAST;
    Position nextposition = {0, 0} ;
    Position currentposition = {0, 0};

    bool operator==(const Step& other) const {
        return (type == other.type &&
                direction == other.direction &&
                nextposition.i == other.nextposition.i &&
                nextposition.j == other.nextposition.j &&
                currentposition.i == other.currentposition.i &&
                currentposition.j == other.currentposition.j);
    }
};
Q_DECLARE_METATYPE(Step)


// Enumeration for the cell state
enum class PieceType {
    NONE,    // No piece
    GREENCAR,
    BLUECAR, // Car piece
    BUS1,
    BUS2,
    BUS3,
    BLACKCAR, // Black car piece
    MOUNT     // Mount piece
};

struct Piece {
    PieceType type = PieceType::NONE; // Default to NONE
    Orientation orientation = Orientation::NONE; // Default orientation
};

struct ActionItem {
    int i;
    Step step;
    PieceType pieceType;

    // Constructor to initialize all members
    ActionItem(int n, const Step& s, PieceType pType) : i(n), step(s), pieceType(pType) {}

    // Default constructor
    ActionItem() : i(0), step(), pieceType(PieceType::NONE) {}
};


// GameBoard class
class TwoPlayerBoard {
private:
    std::array<std::array<Piece, GRID_SIZE>, GRID_SIZE> grid;
    int blackCars; // Count of black cars

    void layoutboard();
    int generateRandomNumber(int min, int max);
    Orientation generateRandomOrientation();
    int move_intermediate_car_bus(Step step);

public:
    TwoPlayerBoard(int blackCars);
    TwoPlayerBoard(const TwoPlayerBoard& originalboard);
    const std::array<std::array<Piece, GRID_SIZE>, GRID_SIZE>& returnGrid() const;
    int MoveCAR(MOVEMENT move, Orientation direction, int initial_row, int initial_column, int destination_row, int destination_column);
    int MoveCar(Step step);
    int MoveBus(Step step, PieceType type);
    int Removecar(Position position);
    bool isValidPosition(int i, int j);
    Orientation calculateOrientation(Position current, Position next);
    Step translationStep(Position current, Position next);
    Step rotationStep(Position current, Orientation currentOrientation);
    Orientation rotateOrientation(Orientation currentOrientation);
    //QMutex mutex;
    bool if_winner_exist=false;
    bool winner; //blue 0 green 1
    int checkwinner();
    bool checkgameover();


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

};


#endif // TWOPLAYERBOARD_H
