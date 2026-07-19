#include "display_control.h"

static uint16_t current_value = 0;

void display_control(uint16_t vp, uint16_t value)
{
    switch (vp)
    {
        case 0x1000:
            switch (value)
            {
                case 0x0001:
                    break;

                case 0x0002:
                    break;

                default:
                    break;
            }
            break;

        case 0x2000:
            current_value = value;
            break;

        default:
            break;
    }
}