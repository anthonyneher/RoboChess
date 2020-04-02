#required modules
from struct import *
import numpy as np

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

def main():
    test_array = np.chararray((4,4))
    for y in range(4):
        for x in range(4):
            if y == 0:
                test_array[y][x] = 'b'
            elif y == 3:
                test_array[y][x] = 'w'
            else: 
                test_array[y][x] = 'e'
    test_array.tostring()
    piece = 'p'
    pickup = 45
    place = 37
    message = test_array.tostring() #+ piece #+ pickup + place
    print(message)

if __name__ == "__main__":
    main()  
