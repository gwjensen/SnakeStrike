#include "utils.h"

void GetColor( int idx, int& oR, int& oG, int& oB )
{
    oR = 0;
    oG = 0;
    oB = 0;
    switch( idx % 4 ){
        case 0:
            oR = 255;
            oG = 0;
            oB = 0;
            break;
        case 1:
            oR = 0;
            oG = 255;
            oB = 0;
            break;
        case 2:
            oR = 0;
            oG = 0;
            oB = 255;
            break;
        case 3:
            oR = 255;
            oG = 0;
            oB = 255;
            break;
    }
}

