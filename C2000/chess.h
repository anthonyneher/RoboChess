/*
 * chess.h
 *
 *  Created on: Mar 16, 2020
 *      Author: drive
 */

#ifndef CHESS_H_
#define CHESS_H_

#include <F28x_Project.h>
#include <F2837xD_Device.h>
#include <LCDLib.h>
#include <stdbool.h>
#include <stdint.h>
#include "STEPPERm_LIB.h"

#define SQUARE_SIDE     36
#define TEXTURE_SIDE    80
#define Y_OFFSET        7
#define X_OFFSET        25
#define BORDER_WIDTH    5

#define BUTTON_LENGTH   130
#define BUTTON_WIDTH    45
#define BUTTON_OFFSET_X MAX_SCREEN_X - BUTTON_LENGTH - 15
#define BUTTON_TEXT_OFF 10

#define PIECE_OFFSET    3
#define PIECE_WIDTH     30

typedef struct chessPiece chessPiece;


#define P   0
#define N   1
#define B   2
#define R   3
#define Q   4
#define K   5
#define S   6

typedef enum{
    BLUE    = 0,
    ORANGE  = 1,
    NONE    = 2
}fl;

struct chessPiece{
    char   piece;
    fl      color;
};

typedef enum{
    X1   = X_OFFSET + PIECE_OFFSET,
    X2   = X_OFFSET + PIECE_OFFSET + SQUARE_SIDE,
    X3   = X_OFFSET + PIECE_OFFSET + 2*SQUARE_SIDE,
    X4   = X_OFFSET + PIECE_OFFSET + 3*SQUARE_SIDE,
    X5   = X_OFFSET + PIECE_OFFSET + 4*SQUARE_SIDE,
    X6   = X_OFFSET + PIECE_OFFSET + 5*SQUARE_SIDE,
    X7   = X_OFFSET + PIECE_OFFSET + 6*SQUARE_SIDE,
    X8   = X_OFFSET + PIECE_OFFSET + 7*SQUARE_SIDE
}xaxis;

typedef enum{
    Y8   = Y_OFFSET + PIECE_OFFSET,
    Y7   = Y_OFFSET + PIECE_OFFSET + SQUARE_SIDE,
    Y6   = Y_OFFSET + PIECE_OFFSET + 2*SQUARE_SIDE,
    Y5   = Y_OFFSET + PIECE_OFFSET + 3*SQUARE_SIDE,
    Y4   = Y_OFFSET + PIECE_OFFSET + 4*SQUARE_SIDE,
    Y3   = Y_OFFSET + PIECE_OFFSET + 5*SQUARE_SIDE,
    Y2   = Y_OFFSET + PIECE_OFFSET + 6*SQUARE_SIDE,
    Y1   = Y_OFFSET + PIECE_OFFSET + 7*SQUARE_SIDE
}yaxis;

extern const char background[12800];
extern const char o_chess_pieces[6][1800];
extern const char b_chess_pieces[6][1800];
extern const char b_PAWN[2450];

/* CHESS LCD FUNCTIONS */
void drawChessBoard();
void initChessPieces(char *pieces);
void drawPiece(uint16_t posX, uint16_t posY, fl fill, char piece);
void drawPieces(char *pieces);
void updatePieces(char *pieces, char *newPieces);


#endif /* CHESS_H_ */
