#pragma once

#ifndef __CONFIGURATION__
    #define __CONFIGURATION__

    #define DEBUG //uncomment to enable serial debug
//    #define DEBUG_ACCELERATION
//    #define DEBUG_ACCEL_STAT
//    #define DEBUG_SPEED
//    #define DEBUG_ALTITUDE
//    #define DEBUG_DRAG
//    #define DEBUG_LOCATION
//    #define DEBUG_SAT
//    #define DEBUG_EEPROM
    #define DEBUG_BATTERY
    #define DEBUG_BAUD      230400

    #define BLINK_PERIOD    1000
    #define SCREEN_REFRESH_PERIOD 500
    #define SIGNAL_HEALTH_PERIOD 5000
    #define DRAG_COUNTDOWN_TIME 6
    #define SLEEP_TIME 300

    #define GPS_RXPIN       25
    #define GPS_TXPIN       26
    #define GPS_POWER_PIN   15

    #define GPS_BAUD        230400

    #define BAT_PIN         34
    #define BAT_READPIN     14
    #define BAT_ON_PIN      12
    #define BAT_CONV_FACTOR 2.04 //1.702
    #define MIN_USB_VOLTS   4.4

    #define BUTTON_A_PIN     0
    #define BUTTON_B_PIN     35

    #define EEPROM_VERSION  3
    #define IMPERIAL 0              //0: metric (kmph and meters) 1: imperial (mph and feet)
    #define SCREEN_OFF_TIME     120
    #define DRAG_START_DISTANCE 10
    #define DRAG_END_DISTANCE 50
    #define ACCELERATION_START_SPEED 5
    #define ACCELERATION_END_SPEED 30

    #define M_TO_KM_FACTOR      1.61
    #define FEET_TO_M_FACTOR    0.305

    #define MIN_TRAVEL_SPEED    3.5

    #define GPS_Connection_Error_Msg "No GPS data received!"

#endif //__CONFIGURATION__