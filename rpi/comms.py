#required modules
from struct import *
import numpy as np
import spidev
import time
import chess

"""
creating bytestream to be sent via SPI
will be recieved into:
struct move{
    char pieces[64];
    char piece_type;
    uint8_t pickup;
    uint8_t place;
}
"""
START = 0x00;
MOVE = 0x01;
DONE_YET = 0x02;
ANY_NEW = 0x03;
BOARD_DISPLAY = 0x04;
TAKE = 0x05;

def main():
    spi = init_spi()
    vhex = np.vectorize(hex)
    test_array = np.chararray((2,2))
    test_array[0][0] = 'a'
    test_array[0][1] = 'b'
    test_array[1][0] = 'c'
    test_array[1][1] = 'd'
    board = chess.Board()
    move = chess.Move.from_uci("d2d3")
    board.push(move)
    while True:
        input("send START")
        Start(spi)
        
        input("send move")
        move = chess.Move.from_uci("e2e3")
        Move(spi, move, chess.QUEEN)
        
        input("send take")
        move = chess.Move.from_uci("d2d3")
        Take(spi, move, chess.PAWN, chess.QUEEN)
        
        input("send move")
        move = chess.Move.from_uci("e2e3")
        Move(spi, move, chess.QUEEN)
            
        
        input("send check for new values command")
        new = CheckNew(spi)
        print("Returned ", new)
        """
        input("send new board display")
        print(board)
        BoardDisplay(spi, board)
        """
        input("send check for new values command")
        new = DoneYet(spi)
        print("Returned ", new)
        
def init_spi():
    spi = spidev.SpiDev()
    spi.open(0,0)
    spi.max_speed_hz = 100000
    spi.mode = 0b01
    return spi
    
#####Command Functions#####
def Start(spi):
    gbg = spi.xfer([START])
    return

def Move(spi, move, place):
    From = move.from_square
    to = move.to_square
    print(place)
    message = [MOVE, From, to, place]
    spi.xfer2(message)
    return
    
def Take(spi, move, place, pickup):
    From = move.from_square
    to = move.to_square
    print(pickup)
    print(place)
    
    message = [TAKE, From, to, place, pickup]
    spi.xfer2(message)
    return
    
def DoneYet(spi):
    message = [DONE_YET, 0]
    data = spi.xfer2(message)
    return data[1]

def CheckNew(spi):
    message = [ANY_NEW, 0]
    data = spi.xfer2(message)
    return data[1]
    
def BoardDisplay(spi, board):
    #in this instance board is a numpy character array
    message = to_char(board)
    message[0] = '\x04'

    print(message)
    spi.writebytes2(message)
    
def to_char(board):
    positions = np.chararray(65)
    for x in range(64):
        piece = board.piece_at(x) 
        if piece is not None:
            positions[x+1] = piece.symbol()
        else:
            positions[x+1] = '.'
    return positions
    


    
if __name__ == "__main__":
    main()  
