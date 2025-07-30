#include "./configuration.h"
#include <Arduino.h>
#include <TFT_eSPI.h>
#include <Battery18650Stats.h>
#include <TinyGPSPlus.h>
//#include <SoftwareSerial.h>
#include <HardwareSerial.h>
//#include <BluetoothSerial.h>
#include "./images.h"
#include <eeprom.h>
#include <Button2.h>


TinyGPSPlus gps;

// The serial connection to the GPS device
//SoftwareSerial ss(GPS_RXPIN, GPS_TXPIN);
HardwareSerial ss(2);


TFT_eSPI tft=TFT_eSPI();
TFT_eSprite GPSinitSprite = TFT_eSprite(&tft);
TFT_eSprite dataSprite = TFT_eSprite(&tft);
TFT_eSprite batSprite = TFT_eSprite(&tft);
TFT_eSprite satSprite = TFT_eSprite(&tft);


Battery18650Stats battery(BAT_PIN,BAT_CONV_FACTOR,10);

u_int8_t batLevel = 50;

int8_t screen_id = 0;

int8_t setting_id = 0;

u_int32_t last_refresh = 0;

int drag_countdown = DRAG_COUNTDOWN_TIME;
u_int32_t drag_count_time = 0;
bool drag_start = 0;

Button2 buttonA, buttonB;

bool screen_off = 0;
u_int32_t last_button_press = 0;

u_int32_t last_move = 0;

char data[50] = {};

typedef struct {
    u_int32_t satellites;

    struct {
        double latitude;
        double longitude;
        u_int32_t upd_time;
    } location;

    struct {
        double latitude;
        double longitude;
    } last_distance;

    struct {
        double mph;
        double kmph;
        double mps;
        u_int32_t upd_time;
    } speed;

    struct {
        double alt_meters;
        double alt_feet;
        u_int32_t upd_time;
    } altitude;

    struct {
        double max_mph;
        double max_kmph;
        double max_alt_meters;
        double max_alt_feet;
        double max_acceleration;
        double max_deceleration;
        float travel_dist;
        float origin_dist;
    } stats;

    struct {
        double latitude;
        double longitude;
        uint32_t upd_time;
    } origin;

    struct {
        u_int32_t start_time;
        u_int32_t end_time;
        u_int32_t start_time_alt;
        u_int32_t end_time_alt;
    } acceleration;

    struct {
        u_int32_t start_time;
        u_int32_t end_time;
        u_int32_t start_time_alt;
        u_int32_t end_time_alt;
    } drag;

} GPS_info_t;

GPS_info_t GPS_info;

typedef struct{
    int version;
    bool imperial;
    int screen_off_time;
    struct {
        int start_speed;
        int end_speed;
    } acceleration;
    struct {
        int start_distance;
        int end_distance;
    } drag;
} config_t;

config_t config_info;

bool readGPS();
void initGPS();
void resetGPS();

void initScreen();

u_int8_t checkBat();

void refreshBat();
void refreshSat();
void updateScreen(int refresh_period);
void screen_settings();
void screen_speed();
void screen_altitude();
void screen_distance();
void screen_location();
void screen_gforce();
void screen_scores();
void screen_drag();

void initConfig();
void putConfig();
//void getConfig();

void clickB(Button2& btn);
void longclickB(Button2& btn);
void doubleclickB(Button2& btn);
void clickA(Button2& btn);
void longclickA(Button2& btn);
void doubleclickA(Button2& btn);

void screenoff();
void screenon();

void sleepGPS();
void wakeup_reason();
void wakeGPS();
void check_activity();

void initConfig(){
    int eepromstat = EEPROM.begin(1024);
    EEPROM.get(0,config_info);
    #ifdef DEBUG_EEPROM
        if (eepromstat) Serial.println("Initialized EEPROM");
            else Serial.println("Failed to initialise EEPROM!!!");
        Serial.print("EEPROM Version ");
        Serial.println(config_info.version);
    #endif

    if (config_info.version != EEPROM_VERSION) {
        config_info.version = EEPROM_VERSION;
        config_info.imperial = IMPERIAL;
        config_info.screen_off_time = SCREEN_OFF_TIME;
        config_info.drag.start_distance = DRAG_START_DISTANCE;
        config_info.drag.end_distance = DRAG_END_DISTANCE;
        config_info.acceleration.start_speed = ACCELERATION_START_SPEED;
        config_info.acceleration.end_speed = ACCELERATION_END_SPEED;
        EEPROM.put(0,config_info);
        EEPROM.commit();
    }
}

void putConfig(){
    if (config_info.drag.start_distance < 5) config_info.drag.start_distance = 5;
    if (config_info.drag.end_distance < config_info.drag.start_distance + 5) config_info.drag.end_distance = config_info.drag.start_distance + 5;
    if (config_info.acceleration.start_speed < 5) config_info.acceleration.start_speed = 5;
    if (config_info.acceleration.end_speed < config_info.acceleration.start_speed + 5) config_info.acceleration.end_speed = config_info.acceleration.start_speed + 5;
    EEPROM.put(0,config_info);
    EEPROM.commit();
}

void resetGPS(){
    GPS_info.location.latitude = 0;
    GPS_info.location.longitude = 0;
    GPS_info.location.upd_time = 0;

    GPS_info.last_distance.latitude = 0;
    GPS_info.last_distance.longitude = 0;

    GPS_info.speed.mph = 0;
    GPS_info.speed.kmph = 0;
    GPS_info.speed.mps = 0;
    GPS_info.speed.upd_time = 0;

    GPS_info.altitude.alt_meters = 0;
    GPS_info.altitude.alt_feet = 0;
    GPS_info.altitude.upd_time = 0;

    GPS_info.stats.max_mph = 0;
    GPS_info.stats.max_kmph = 0;
    GPS_info.stats.max_alt_meters = 0;
    GPS_info.stats.max_alt_feet = 0;
    GPS_info.stats.max_acceleration = 0;
    GPS_info.stats.max_deceleration = 0;
    GPS_info.stats.travel_dist = 0;
    GPS_info.stats.origin_dist = 0;

    GPS_info.origin.latitude = 0;
    GPS_info.origin.longitude = 0;
    GPS_info.origin.upd_time = 0;

    GPS_info.acceleration.start_time = 0;
    GPS_info.acceleration.end_time = 0;

    GPS_info.drag.start_time = 0;
    GPS_info.drag.end_time = 0;

    GPS_info.acceleration.start_time_alt = 0;
    GPS_info.acceleration.end_time_alt = 0;

    GPS_info.drag.start_time_alt = 0;
    GPS_info.drag.end_time_alt = 0;
}

void sleepGPS()
{
    digitalWrite(GPS_POWER_PIN, LOW); // Turn off GPS receiver
    tft.fillScreen(TFT_BLACK); // clear screen:
    digitalWrite(TFT_BL, LOW); // backlight OFF
    tft.writecommand(0x10); // SLEEP
    delay(50); // needed!
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0,0); //configure wakeup source
    delay(1000);
    esp_deep_sleep_start(); // sleep
}

void check_activity()
{
    u_int32_t curtime = millis();

    if ((GPS_info.speed.kmph>0) || (last_move > curtime)) last_move = curtime;

    if (((curtime > (last_move + SLEEP_TIME * 1000)) || (curtime > (GPS_info.speed.upd_time + SLEEP_TIME * 1000))) && (curtime > (last_button_press + SLEEP_TIME * 1000))) sleepGPS(); //in last 10 minutes no positive speed or no gps update on speed and no button press
}

void wakeGPS()
{
//    digitalWrite(GPS_POWER_PIN, HIGH);
    digitalWrite(TFT_BL, HIGH); // backlight ON
    tft.writecommand(0x11); // WAKEUP
    delay(200); // needed! PWR neeeds to stabilize!
    
}

void wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : 
        //Serial.println("Wakeup caused by external signal using RTC_IO");
        wakeGPS();
    break;
    //case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    //case ESP_SLEEP_WAKEUP_TIMER : 
    //  Serial.println("> Wakeup caused by timer"); 
    //  TFT_wake();
    //break;
    //
    //case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    //case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    //default : 
    //Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); 
    //break;
  }
}

void initGPS()
{
    pinMode(GPS_POWER_PIN, OUTPUT);
    digitalWrite(GPS_POWER_PIN, HIGH);

    //ss.begin(GPS_BAUD);
    ss.begin(GPS_BAUD, SERIAL_8N1, GPS_RXPIN, GPS_TXPIN);

    resetGPS();

    uint32_t last_blink = 0;
    uint32_t time;
    bool state = 1;

    //TFT_eSprite GPSinitSprite = TFT_eSprite(&tft);

    GPSinitSprite.createSprite(45,45);
    GPSinitSprite.setSwapBytes(true);
    GPSinitSprite.fillSprite(TFT_BLACK);

    last_blink = 0;

    /*do 
    {
        time = millis();

        if ((time - BLINK_PERIOD) > last_blink)
        {
            if (state)
            {
                GPSinitSprite.pushImage(0,0,45,45,(uint16_t *) satellite_dish_45x45);
                GPSinitSprite.pushSprite(45,97);
            } else {
                GPSinitSprite.fillSprite(TFT_BLACK);
                GPSinitSprite.pushSprite(45,97);
            }
            last_blink = time;
            state = !state;
        }

        if (time < last_blink) last_blink = time;

        check_activity();

    } while (!readGPS());*/

    #ifdef GPS_ATGM336H
        ss.print("$PCAS02,100*1E\r\n");  //10hz
        delay(1000);

        ss.print("$PCAS11,3*1E\r\n");;   //Automotive mode
        delay(1000);

        ss.print("$PCAS01,5*19\r\n");;   //115200 baud
        delay(1000);

        ss.end();
        delay(1000);

        ss.begin(115200, SERIAL_8N1, GPS_RXPIN, GPS_TXPIN);

        resetGPS();
    #endif

    GPSinitSprite.fillSprite(TFT_BLACK);
    GPSinitSprite.pushSprite(45,97);
    GPSinitSprite.deleteSprite();
}

bool readGPS()
{
    unsigned long start = millis();
    double accel = 0;
    bool result = false;

    while (ss.available() > 0)
    {
   //         Serial.println(ss.read());
            gps.encode(ss.read());
    }

    if (gps.satellites.isUpdated()){
        GPS_info.satellites = gps.satellites.value();

        #ifdef DEBUG_SAT
            Serial.print(F("Satellites = "));
            Serial.println(GPS_info.satellites);
        #endif
    }

    if (gps.speed.isUpdated()) {

        #ifdef DEBUG_SPEED
            Serial.print(F("SPEED      Fix Age="));
            Serial.print(gps.speed.age());
            Serial.print(F(" MPH="));
            Serial.print(gps.speed.mph());
            Serial.print(F(" KMPH="));
            Serial.print(gps.speed.kmph());
            Serial.print(F(" MPS="));
            Serial.println(gps.speed.mps());
        #endif
        
        if ((GPS_info.speed.upd_time >0) && ((millis() - gps.speed.age() - GPS_info.speed.upd_time) > 0)) {
             
            GPS_info.stats.max_kmph = max(GPS_info.stats.max_kmph, gps.speed.kmph());
            GPS_info.stats.max_mph = max(GPS_info.stats.max_mph, gps.speed.mph());
                
            accel = ((gps.speed.mps() - GPS_info.speed.mps) / (double) ((millis()-gps.speed.age()) - (GPS_info.speed.upd_time))) * 1000;
            GPS_info.stats.max_acceleration = max(GPS_info.stats.max_acceleration, accel);
            GPS_info.stats.max_deceleration = min(GPS_info.stats.max_deceleration, accel);

            #ifdef DEBUG_ACCELERATION
                Serial.print(F("SPEED-MAX  Max KMPH="));
                Serial.print(GPS_info.stats.max_kmph);
                Serial.print(F(" Max MPH="));
                Serial.print(GPS_info.stats.max_mph);
                Serial.print(F(" INFO MPS="));
                Serial.print(GPS_info.speed.mps);
                Serial.print(F(" GPS MPS="));
                Serial.print(gps.speed.mps());
                Serial.print(F(" MAX ACCEL MPS="));
                Serial.print(GPS_info.stats.max_acceleration);
                Serial.print(F(" MAX DECEL MPS="));
                Serial.println(GPS_info.stats.max_deceleration);
            #endif
        }

        GPS_info.speed.kmph = gps.speed.kmph();
        GPS_info.speed.mph = gps.speed.mph();
        GPS_info.speed.mps = gps.speed.mps();
        GPS_info.speed.upd_time = millis() - gps.speed.age();

        if ((GPS_info.acceleration.start_time == 0) && (config_info.acceleration.start_speed < GPS_info.speed.kmph))
            GPS_info.acceleration.start_time = GPS_info.speed.upd_time;

        if ((GPS_info.acceleration.end_time == 0) && (config_info.acceleration.end_speed < GPS_info.speed.kmph))
            GPS_info.acceleration.end_time = GPS_info.speed.upd_time;

        if ((GPS_info.acceleration.end_time_alt == 0) && (GPS_info.acceleration.start_time_alt > 0) && (drag_start == 1) && (drag_countdown < 1) && (config_info.acceleration.end_speed < GPS_info.speed.kmph))
        {
            GPS_info.acceleration.end_time_alt = GPS_info.speed.upd_time;
            if (GPS_info.drag.end_time_alt > 0) 
                {
                    drag_start = 0;
                    drag_countdown = DRAG_COUNTDOWN_TIME;
                    drag_count_time = 0;
                }
        }

        #ifdef DEBUG_ACCEL_STAT
            Serial.print(F("SPEED-TIME UPD TIME="));
            Serial.print(GPS_info.speed.upd_time);
            Serial.print(F(" ACC. START TIME="));
            Serial.print(GPS_info.acceleration.start_time);
            Serial.print(F(" ACC. END TIME="));
            Serial.print(GPS_info.acceleration.end_time);
            Serial.print(F(" ALT ACC. START TIME="));
            Serial.print(GPS_info.acceleration.start_time_alt);
            Serial.print(F(" ALT ACC. END TIME="));
            Serial.println(GPS_info.acceleration.end_time_alt);
        #endif

        result = true;
    }

    if (gps.location.isUpdated())
    {
        #ifdef DEBUG_LOCATION
            Serial.print(F("LOCATION   Fix Age="));
            Serial.print(gps.location.age());
            Serial.print(F("  Lat="));
            Serial.print(gps.location.lat(), 6);
            Serial.print(F(" Long="));
            Serial.println(gps.location.lng(), 6);
        #endif

        GPS_info.location.upd_time = (u_int32_t) millis() - (u_int32_t) gps.location.age();

        if (GPS_info.origin.upd_time == 0) {
            GPS_info.origin.latitude = gps.location.lat();
            GPS_info.origin.longitude = gps.location.lng();
            GPS_info.origin.upd_time = GPS_info.location.upd_time;

            GPS_info.last_distance.latitude = GPS_info.origin.latitude;
            GPS_info.last_distance.longitude = GPS_info.origin.longitude;
        } else {
            if ((gps.speed.kmph() > MIN_TRAVEL_SPEED) && (gps.speed.isValid())) {
                GPS_info.stats.travel_dist += gps.distanceBetween(gps.location.lat(),gps.location.lng(),
                                                                  GPS_info.last_distance.latitude,GPS_info.last_distance.longitude);
                GPS_info.last_distance.latitude = gps.location.lat();
                GPS_info.last_distance.longitude = gps.location.lng();
            }
            GPS_info.stats.origin_dist = gps.distanceBetween(gps.location.lat(),gps.location.lng(),
                                                             GPS_info.origin.latitude,GPS_info.origin.longitude);

            if ((GPS_info.drag.start_time == 0) && (config_info.drag.start_distance < GPS_info.stats.origin_dist))
                GPS_info.drag.start_time = GPS_info.location.upd_time;

            if ((GPS_info.drag.end_time == 0) && (config_info.drag.end_distance < GPS_info.stats.origin_dist))
                GPS_info.drag.end_time = GPS_info.location.upd_time;

            if ((GPS_info.drag.end_time_alt == 0) && (GPS_info.drag.start_time_alt > 0) && (drag_start == 1) && (drag_countdown < 1) && (config_info.drag.end_distance < GPS_info.stats.origin_dist))
            {
                GPS_info.drag.end_time_alt = GPS_info.location.upd_time;
                if (GPS_info.acceleration.end_time_alt > 0) 
                {
                    drag_start = 0;
                    drag_countdown = DRAG_COUNTDOWN_TIME;
                    drag_count_time = 0;
                }
            }

            #ifdef DEBUG_DRAG
                Serial.print(F("DRAG  ORIGIN DIST.="));
                Serial.print(GPS_info.stats.origin_dist);
                Serial.print(F(" LOC. UPD. TIME="));
                Serial.print(GPS_info.location.upd_time);
                Serial.print(F(" DRAG START TIME="));
                Serial.print(GPS_info.drag.start_time);
                Serial.print(F(" DRAG END TIME="));
                Serial.print(GPS_info.drag.end_time);
                Serial.print(F(" ALT DRAG START TIME="));
                Serial.print(GPS_info.drag.start_time_alt);
                Serial.print(F(" ALT DRAG END TIME="));
                Serial.println(GPS_info.drag.end_time_alt);
            #endif
        }

        GPS_info.location.latitude = gps.location.lat();
        GPS_info.location.longitude = gps.location.lng();

    }


    

    if (gps.altitude.isUpdated()) {

        #ifdef DEBUG_ALTITUDE
            Serial.print(F("Altitude   Fix Age="));
            Serial.print(gps.altitude.age());
            Serial.print(F(" meters="));
            Serial.print(gps.altitude.meters());
            Serial.print(F(" feet="));
            Serial.println(gps.altitude.feet());
        #endif

        GPS_info.stats.max_alt_feet = max(GPS_info.stats.max_alt_feet, gps.altitude.feet());
        GPS_info.stats.max_alt_meters = max(GPS_info.stats.max_alt_meters, gps.altitude.meters());
        
        GPS_info.altitude.alt_feet = gps.altitude.feet();
        GPS_info.altitude.alt_meters = gps.altitude.meters();
        GPS_info.altitude.upd_time = millis() - gps.altitude.age();
    }


    if (gps.location.isValid()) result = true;



    return result;
}


void initScreen()
{

    tft.init();
    tft.setRotation(0);
    tft.setSwapBytes(true);
    tft.fillScreen(TFT_BLACK);

    dataSprite.createSprite(135,240);
    dataSprite.setSwapBytes(true);
    dataSprite.fillSprite(TFT_BLACK);

    batSprite.createSprite(40,30);
    batSprite.setSwapBytes(true);
    batSprite.fillSprite(TFT_BLACK);

    satSprite.createSprite(95,30);
    //satSprite.setSwapBytes(true);
    satSprite.fillSprite(TFT_BLACK);
 
  
    pinMode(BAT_READPIN, OUTPUT);
    pinMode(BAT_PIN,INPUT);
    pinMode(BAT_ON_PIN, INPUT_PULLUP);
    digitalWrite(BAT_READPIN, HIGH);
    batLevel = 50;

    buttonA.begin(BUTTON_A_PIN);
    buttonA.setClickHandler(clickA);
    buttonA.setLongClickDetectedHandler(longclickA);
    buttonA.setDoubleClickHandler(doubleclickA);

    buttonA.setDebounceTime(10);
    buttonA.setLongClickTime(1200);
    buttonA.setDoubleClickTime(500);


    buttonB.begin(BUTTON_B_PIN);
    buttonB.setClickHandler(clickB);
    buttonB.setLongClickDetectedHandler(longclickB);
    buttonB.setDoubleClickHandler(doubleclickB);

    buttonB.setDebounceTime(10);
    buttonB.setLongClickTime(1200);
    buttonA.setDoubleClickTime(500);

    screen_id = 1;
    setting_id = 0;

    last_button_press = millis();

    #ifdef DEBUG
        Serial.println("***Screen initialization complete!***");
    #endif


}

void clickB(Button2& btn) {
    if (screen_off) screenon();
    else {

    if (((screen_id > 0) && (screen_id < 7)) || ((setting_id == 0) && (screen_id == 0)) || ((screen_id == 7) && (drag_start == 0))) screen_id++;

    if (screen_id > 7) screen_id = 0;

    if (screen_id == 0) {
        switch (setting_id) {
            case 1:
                config_info.imperial = !config_info.imperial;
            break;
            case 2:
                config_info.drag.start_distance++;
                if (config_info.drag.end_distance < config_info.drag.start_distance + 5) config_info.drag.end_distance = config_info.drag.start_distance + 5;
            break;
            case 3:
                config_info.drag.end_distance++;
            break;
            case 4:
                config_info.acceleration.start_speed++;
                if (config_info.acceleration.end_speed < config_info.acceleration.start_speed + 5) config_info.acceleration.end_speed = config_info.acceleration.start_speed + 5;
            break;
            case 5:
                config_info.acceleration.end_speed++;
            break;
            case 6:
                config_info.screen_off_time++;
                if (config_info.screen_off_time > 600) config_info.screen_off_time = 600;
            break;
        }
    }
    }
    last_button_press = millis();
    updateScreen(0);
}

void longclickB(Button2& btn) {
        if (screen_off) screenon();
    else {
    if ((screen_id > 0) && (screen_id < 7)) resetGPS();
        else if (screen_id == 0)
        {
            putConfig();
            screen_id++;
            setting_id = 0;
        }
        else if (screen_id == 7) 
        {
            if (!drag_start)
            {
                drag_start = 1;
                drag_countdown = DRAG_COUNTDOWN_TIME;
                drag_count_time = millis();
                GPS_info.acceleration.start_time_alt = 0;
                GPS_info.acceleration.end_time_alt = 0;
                GPS_info.drag.start_time_alt = 0;
                GPS_info.drag.end_time_alt = 0;
            }
        }
    }
    last_button_press = millis();
    updateScreen(0);
}

void doubleclickB(Button2& btn) {
        if (screen_off) screenon();
    else {
        if (screen_id == 0) {
        switch (setting_id) {
            case 2:
                config_info.drag.start_distance +=5;
                if (config_info.drag.end_distance < config_info.drag.start_distance + 5) config_info.drag.end_distance = config_info.drag.start_distance + 5;
            break;
            case 3:
                config_info.drag.end_distance +=5;
            break;
            case 4:
                config_info.acceleration.start_speed +=5;
                if (config_info.acceleration.end_speed < config_info.acceleration.start_speed + 5) config_info.acceleration.end_speed = config_info.acceleration.start_speed + 5;
            break;
            case 5:
                config_info.acceleration.end_speed +=5;
            break;
            case 6:
                config_info.screen_off_time +=5;
                if (config_info.screen_off_time > 600) config_info.screen_off_time = 600;
            break;
        }
    }
    }
    last_button_press = millis();
    updateScreen(0);

}

void clickA(Button2& btn) {
        if (screen_off) screenon();
    else {
    if (screen_id == 0){
        switch (setting_id) {
            case 1:
                config_info.imperial = !config_info.imperial;
            break;
            case 2:
                config_info.drag.start_distance--;
                if (config_info.drag.start_distance < 5) config_info.drag.start_distance = 5;
            break;
            case 3:
                config_info.drag.end_distance--;
                if (config_info.drag.end_distance < config_info.drag.start_distance + 5) config_info.drag.end_distance = config_info.drag.start_distance + 5;
            break;
            case 4:
                config_info.acceleration.start_speed--;
                if (config_info.acceleration.start_speed < 5) config_info.acceleration.start_speed = 5;
            break;
            case 5:
                config_info.acceleration.end_speed--;
                if (config_info.acceleration.end_speed < config_info.acceleration.start_speed + 5) config_info.acceleration.end_speed = config_info.acceleration.start_speed + 5;
            break;
            case 6:
                config_info.screen_off_time--;
                if (config_info.screen_off_time < 0) config_info.screen_off_time = 0;
            break;
        }
    }
    
    if (((screen_id > 0) && (screen_id < 7)) || ((setting_id == 0) && (screen_id == 0)) || ((screen_id == 7) && (drag_start == 0))) screen_id--;
    if (screen_id < 0) screen_id = 7;
    }
    last_button_press = millis();
    updateScreen(0);
}

void longclickA(Button2& btn) {
        if (screen_off) screenon();
    else {
    if ((screen_id > 0) && (screen_id <7)) config_info.imperial = !config_info.imperial;
        else if (screen_id == 0)
        {
            setting_id++;
            if (setting_id > 6) setting_id = 0;
        }
        else if (screen_id == 7) 
        {
            if (drag_start)
            {
                drag_start = 0;
                drag_countdown = DRAG_COUNTDOWN_TIME;
                drag_count_time = 0;
                GPS_info.acceleration.start_time_alt = 0;
                GPS_info.acceleration.end_time_alt = 0;
                GPS_info.drag.start_time_alt = 0;
                GPS_info.drag.end_time_alt = 0;
            }
        }
    }
    last_button_press = millis();
    updateScreen(0);
}

void doubleclickA(Button2& btn) {
        if (screen_off) screenon();
    else {
        if (screen_id == 0) {
        switch (setting_id) {
            case 2:
                config_info.drag.start_distance -=5;
                if (config_info.drag.start_distance < 5) config_info.drag.start_distance = 5;
            break;
            case 3:
                config_info.drag.end_distance -=5;
                if (config_info.drag.end_distance < config_info.drag.start_distance + 5) config_info.drag.end_distance = config_info.drag.start_distance + 5;
            break;
            case 4:
                config_info.acceleration.start_speed -=5;
                if (config_info.acceleration.start_speed < 5) config_info.acceleration.start_speed = 5;
            break;
            case 5:
                config_info.acceleration.end_speed -=5;
                if (config_info.acceleration.end_speed < config_info.acceleration.start_speed + 5) config_info.acceleration.end_speed = config_info.acceleration.start_speed + 5;
            break;
            case 6:
                config_info.screen_off_time -=5;
                if (config_info.screen_off_time < 0) config_info.screen_off_time = 0;
            break;
        }
    }
    }
    last_button_press = millis();
    updateScreen(0);
}

void refreshBat(){
    batLevel = battery.getBatteryChargeLevel(true);   //checkBat();
    batSprite.fillSprite(TFT_BLACK);
    
    #ifdef DEBUG_BATTERY
        Serial.print("BAT Analog Read ");
        Serial.println(analogRead(BAT_PIN));
        Serial.print("BAT Charge Level ");
        Serial.println(battery.getBatteryChargeLevel(true));
        Serial.print("BAT Volts ");
        Serial.println(battery.getBatteryVolts());
        Serial.print("BAT ON PIN ");
        Serial.println(digitalRead(BAT_ON_PIN));
    #endif
    
    if (battery.getBatteryVolts() > MIN_USB_VOLTS)
    {
        batSprite.drawCentreString(F("CHRG"),20,7,2);
    }
        else {
            batSprite.pushImage(2,5,36,20,(uint16_t *) battery_36X20);
            if (digitalRead(BAT_ON_PIN)) 
            {
                batSprite.fillRect(8,11,map(batLevel,0,100,0,22),8,TFT_WHITE);
                //batSprite.drawNumber(batLevel,5,7,2);
                //batSprite.drawCentreString(String(batLevel),20,7,2);
            }
                else {   
                    batSprite.drawLine(2,5,38,25,TFT_WHITE);
                    batSprite.drawLine(38,5,2,25,TFT_WHITE);
                }
        }
}


void refreshSat(){
    satSprite.fillSprite(TFT_BLACK);

    if (!config_info.imperial) strcpy(data,"SI");
        else strcpy(data,"IMP");
    
    // if bluetooth connected
    //satSprite.pushImage(0,2,20,26,(uint16_t *) bluetooth_20X26);

    if ((millis() - GPS_info.location.upd_time) < SIGNAL_HEALTH_PERIOD)
        satSprite.pushImage(24,2,26,26,(uint16_t *) satellite_dish_26x26);
        else satSprite.pushImage(24,2,26,26,(uint16_t *) satellite_dish_RED_26x26);

    int16_t width = satSprite.drawNumber(GPS_info.satellites,52,7,2);

    satSprite.drawRightString(data,92,7,2);

/*
  satSprite.pushImage(2,2,26,26,(uint16_t *) satellite_dish_26x26);
  int16_t width = satSprite.drawNumber(GPS_info.satellites,31,7,2);

 // if bluetooth connected
  satSprite.pushImage(31+width+4,2,20,26,(uint16_t *) bluetooth_20X26);*/
}

u_int8_t checkBat()
{
    #ifdef DEBUG_BATTERY
        Serial.print("BAT Analog Read ");
        Serial.println(analogRead(BAT_PIN));
        Serial.print("BAT Charge Level ");
        Serial.println(battery.getBatteryChargeLevel(true));
        Serial.print("BAT Volts ");
        Serial.println(battery.getBatteryVolts());
    #endif

    //Charging
    if (analogRead(BAT_PIN) > 2700) return 110;
        else if ((analogRead(BAT_PIN) > 2510) && (analogRead(BAT_PIN) <= 2700)) return 105;
        else return (int) ((battery.getBatteryChargeLevel(true) * 1.1) - 10);

}

void screen_settings()
{
            dataSprite.drawFastHLine(0,0,135,TFT_WHITE);
            dataSprite.drawCentreString(F(LANG_SETTINGS),67,3,4);
            dataSprite.drawFastHLine(0,27,135,TFT_WHITE);

            switch (setting_id) {
                case 1:
                    dataSprite.fillRect(0,27,135,24,TFT_DARKGREEN);
                    break;
                case 2:
                    dataSprite.fillRect(0,53,135,24,TFT_DARKGREEN);
                    break;
                case 3:
                    dataSprite.fillRect(0,79,135,24,TFT_DARKGREEN);
                    break;
                case 4:
                    dataSprite.fillRect(0,105,135,24,TFT_DARKGREEN);
                    break;
                case 5:
                    dataSprite.fillRect(0,131,135,24,TFT_DARKGREEN);
                    break;
                case 6:
                    dataSprite.fillRect(0,157,135,24,TFT_DARKGREEN);
                    break;
/*                case 7:
                    dataSprite.fillRect(0,183,135,24,TFT_DARKGREEN);
                    break;*/
           }

            if (setting_id == 1) dataSprite.setTextColor(TFT_WHITE,TFT_DARKGREEN);
                else dataSprite.setTextColor(TFT_WHITE,TFT_BLACK);
            dataSprite.drawString(F(LANG_IMP),0,30,4);
            sprintf(data,"%d",(int) config_info.imperial);
            dataSprite.drawRightString(data,135,30,4);
            dataSprite.drawFastHLine(0,53,135,TFT_WHITE);

            /*dataSprite.setTextColor(TFT_WHITE,TFT_BLACK);
            dataSprite.drawCentreString(F("Drag"),67,56,4);
            dataSprite.drawFastHLine(0,79,135,TFT_WHITE);*/

            if (setting_id == 2) dataSprite.setTextColor(TFT_WHITE,TFT_DARKGREEN);
                else dataSprite.setTextColor(TFT_WHITE,TFT_BLACK);
            dataSprite.drawString(F(LANG_DSTART),0,56,4);
            sprintf(data,"%d",(int) config_info.drag.start_distance);
            dataSprite.drawRightString(data,135,56,4);
            dataSprite.drawFastHLine(0,79,135,TFT_WHITE);

            if (setting_id == 3) dataSprite.setTextColor(TFT_WHITE,TFT_DARKGREEN);
                else dataSprite.setTextColor(TFT_WHITE,TFT_BLACK);
            dataSprite.drawString(F(LANG_DEND),0,82,4);
            sprintf(data,"%d",(int) config_info.drag.end_distance);
            dataSprite.drawRightString(data,135,82,4);
            dataSprite.drawFastHLine(0,105,135,TFT_WHITE);

            /*dataSprite.setTextColor(TFT_WHITE,TFT_BLACK);
            dataSprite.drawCentreString(F("Accel."),67,134,4);
            dataSprite.drawFastHLine(0,157,135,TFT_WHITE);*/

            if (setting_id == 4) dataSprite.setTextColor(TFT_WHITE,TFT_DARKGREEN);
                else dataSprite.setTextColor(TFT_WHITE,TFT_BLACK);
            dataSprite.drawString(F(LANG_ASTART),0,108,4);
            sprintf(data,"%d",(int) config_info.acceleration.start_speed);
            dataSprite.drawRightString(data,135,108,4);
            dataSprite.drawFastHLine(0,131,135,TFT_WHITE);

            if (setting_id == 5) dataSprite.setTextColor(TFT_WHITE,TFT_DARKGREEN);
                else dataSprite.setTextColor(TFT_WHITE,TFT_BLACK);
            dataSprite.drawString(F(LANG_AEND),0,134,4);
            sprintf(data,"%d",(int) config_info.acceleration.end_speed);
            dataSprite.drawRightString(data,135,134,4);
            dataSprite.drawFastHLine(0,157,135,TFT_WHITE);

            if (setting_id == 6) dataSprite.setTextColor(TFT_WHITE,TFT_DARKGREEN);
                else dataSprite.setTextColor(TFT_WHITE,TFT_BLACK);
            dataSprite.drawString(F(LANG_TIMER),0,160,4);
            sprintf(data,"%d",(int) config_info.screen_off_time);
            dataSprite.drawRightString(data,135,160,4);
            dataSprite.drawFastHLine(0,183,135,TFT_WHITE);

            dataSprite.fillRect(0,184,135,25,TFT_DARKGREY);
            dataSprite.setTextColor(TFT_WHITE,TFT_DARKGREY);
            sprintf(data,"%d%%",battery.getBatteryChargeLevel(true));
            dataSprite.drawString(data,0,186,4);
            sprintf(data,"%.2fV",battery.getBatteryVolts());
            dataSprite.drawRightString(data,135,186,4);

            dataSprite.drawFastHLine(0,210,135,TFT_WHITE);
}

void screen_speed()
{
            dataSprite.drawFastHLine(0,0,135,TFT_WHITE);
            dataSprite.drawCentreString(F(LANG_SPEED),67,3,4);
            dataSprite.drawFastHLine(0,27,135,TFT_WHITE);
            

            if (!config_info.imperial) strcpy(data,"kmph");
                else strcpy(data,"mph");

            dataSprite.drawString(F(LANG_CUR),0,34,4);
            dataSprite.drawRightString(data,135,42,2);
            //width = dataSprite.drawNumber(GPS_info.speed.kmph,0,68,6);
            dataSprite.drawFastHLine(0,120,135,TFT_WHITE);

            dataSprite.drawString(F(LANG_MAX),0,127,4);
            dataSprite.drawRightString(data,135,135,2);
            //width = dataSprite.drawNumber(GPS_info.stats.max_kmph,0,161,6);
            dataSprite.drawFastHLine(0,210,135,TFT_WHITE);

            if (!config_info.imperial) sprintf(data,"%d",(int) GPS_info.speed.kmph);
                else sprintf(data,"%d",(int) GPS_info.speed.mph);

            dataSprite.drawRightString(data,135,68,6);

            if (!config_info.imperial) sprintf(data,"%d",(int) GPS_info.stats.max_kmph);
                else sprintf(data,"%d",(int) GPS_info.stats.max_mph);

            dataSprite.drawRightString(data,135,161,6);
}

void screen_altitude()
{
            dataSprite.drawFastHLine(0,0,135,TFT_WHITE);
            dataSprite.drawCentreString(F(LANG_ALTITUDE),67,3,4);
            dataSprite.drawFastHLine(0,27,135,TFT_WHITE);
            

            if (!config_info.imperial) strcpy(data,"meters");
                else strcpy(data,"feet");

            dataSprite.drawString(F(LANG_CUR),0,34,4);
            dataSprite.drawRightString(data,135,42,2);
            dataSprite.drawFastHLine(0,120,135,TFT_WHITE);

            dataSprite.drawString(F(LANG_MAX),0,127,4);
            dataSprite.drawRightString(data,135,135,2);
            dataSprite.drawFastHLine(0,210,135,TFT_WHITE);

            if (!config_info.imperial) sprintf(data,"%d",(int) GPS_info.altitude.alt_meters);
                else sprintf(data,"%d",(int) GPS_info.altitude.alt_feet);

            dataSprite.drawRightString(data,135,68,6);

            if (!config_info.imperial) sprintf(data,"%d",(int) GPS_info.stats.max_alt_meters);
                else sprintf(data,"%d",(int) GPS_info.stats.max_alt_feet);
                
            dataSprite.drawRightString(data,135,161,6);
}

void screen_distance()
{
            dataSprite.drawFastHLine(0,0,135,TFT_WHITE);
            dataSprite.drawCentreString(F(LANG_DISTANCE),67,3,4);
            dataSprite.drawFastHLine(0,27,135,TFT_WHITE);
            

            if (!config_info.imperial) strcpy(data,"meters");
                else strcpy(data,"feet");

            dataSprite.drawString(F(LANG_ORIGIN),0,34,4);
            dataSprite.drawRightString(data,135,42,2);
            //width = dataSprite.drawNumber(GPS_info.speed.kmph,0,68,6);
            dataSprite.drawFastHLine(0,120,135,TFT_WHITE);

            dataSprite.drawString(F(LANG_TRAVEL),0,127,4);
            dataSprite.drawRightString(data,135,135,2);
            //width = dataSprite.drawNumber(GPS_info.stats.max_kmph,0,161,6);
            dataSprite.drawFastHLine(0,210,135,TFT_WHITE);

            if (!config_info.imperial) sprintf(data,"%d",(int) GPS_info.stats.origin_dist);
                else sprintf(data,"%d",(int) round(GPS_info.stats.origin_dist / FEET_TO_M_FACTOR));

            dataSprite.drawRightString(data,135,68,6);

            if (!config_info.imperial) sprintf(data,"%d",(int) GPS_info.stats.travel_dist);
                else sprintf(data,"%d",(int) round(GPS_info.stats.travel_dist / FEET_TO_M_FACTOR));

            dataSprite.drawRightString(data,135,161,6);
}

void screen_location()
{
            dataSprite.drawFastHLine(0,0,135,TFT_WHITE);
            dataSprite.drawCentreString(F(LANG_LOCATION),67,3,4);
            dataSprite.drawFastHLine(0,27,135,TFT_WHITE);

            dataSprite.drawString(F(LANG_LATITUDE),0,34,4);
            sprintf(data, "%.6lf" ,GPS_info.location.latitude);
            dataSprite.drawString(data,0,90,4);//68
            dataSprite.drawFastHLine(0,120,135,TFT_WHITE);

            dataSprite.drawString(F(LANG_LONGITUDE),0,127,4);
            sprintf(data, "%.6lf" ,GPS_info.location.longitude);
            dataSprite.drawString(data,0,183,4);//161
           // width = dataSprite.drawNumber(GPS_info.location.longitude,0,161,6);
            dataSprite.drawFastHLine(0,210,135,TFT_WHITE);
}

void screen_gforce()
{
            dataSprite.drawFastHLine(0,0,135,TFT_WHITE);
            dataSprite.drawCentreString(F(LANG_GFORCE),67,3,4);
            dataSprite.drawFastHLine(0,27,135,TFT_WHITE);

            dataSprite.drawString(F(LANG_ACC),0,34,4);
            dataSprite.drawFastHLine(0,120,135,TFT_WHITE);

            dataSprite.drawString(F(LANG_DEC),0,127,4);
            dataSprite.drawFastHLine(0,210,135,TFT_WHITE);

            sprintf(data, "%.2f" ,(double) (GPS_info.stats.max_acceleration / 9.81));

            dataSprite.drawRightString(data,135,68,6);

            sprintf(data, "%.2f" ,(double) (GPS_info.stats.max_deceleration / 9.81));

            dataSprite.drawRightString(data,135,161,6);
}

void screen_scores()
{
            dataSprite.drawFastHLine(0,0,135,TFT_WHITE);
            dataSprite.drawCentreString(F(LANG_SCORES),67,3,4);
            dataSprite.drawFastHLine(0,27,135,TFT_WHITE);
            

            if (!config_info.imperial) sprintf(data,"%d-%dmeters",(int) config_info.drag.start_distance,(int) config_info.drag.end_distance);
                else sprintf(data,"%d-%dfeet",(int) round(config_info.drag.start_distance / FEET_TO_M_FACTOR),(int) round(config_info.drag.end_distance / FEET_TO_M_FACTOR));

            dataSprite.drawString(F(LANG_DRAG),0,34,4);
            dataSprite.drawRightString(data,135,42,2);
            //width = dataSprite.drawNumber(GPS_info.speed.kmph,0,68,6);
            dataSprite.drawFastHLine(0,120,135,TFT_WHITE);

            if (!config_info.imperial) sprintf(data,"%d-%dkmph",(int) config_info.acceleration.start_speed,(int) config_info.acceleration.end_speed);
                else sprintf(data,"%d-%dmph",(int) round(config_info.acceleration.start_speed / M_TO_KM_FACTOR),(int) round(config_info.acceleration.end_speed / M_TO_KM_FACTOR));

            dataSprite.drawString(F(LANG_ACC2),0,127,4);
            dataSprite.drawRightString(data,135,135,2);
            //width = dataSprite.drawNumber(GPS_info.stats.max_kmph,0,161,6);
            dataSprite.drawFastHLine(0,210,135,TFT_WHITE);

            double scoretime = (double) (GPS_info.drag.end_time - GPS_info.drag.start_time) / 1000.0;

            int textsize;

            textsize = (scoretime >= 100.0) ? 4 : 6; 

            if (GPS_info.drag.end_time == 0) 
            {
                strcpy(data,"--");
                textsize = 6;
            }
                else sprintf(data, "%.2f" ,(double) scoretime);

            dataSprite.drawRightString(data,135,68,textsize);

            scoretime = (double) (GPS_info.acceleration.end_time - GPS_info.acceleration.start_time) / 1000.0;

            textsize = (scoretime >= 100.0) ? 4 : 6; 

            if (GPS_info.acceleration.end_time == 0)
            {
                strcpy(data,"--");
                textsize = 6;
            }
                else sprintf(data, "%.2f" ,(double) scoretime);

            dataSprite.drawRightString(data,135,161,textsize);
}

void screen_drag()
{
    if ((!drag_start) || (drag_countdown == -3))
    {
            dataSprite.drawFastHLine(0,0,135,TFT_WHITE);

            if (drag_start) dataSprite.setTextColor(TFT_RED,TFT_BLACK);

            dataSprite.drawCentreString(F(LANG_0DRAG),67,3,4);
            dataSprite.drawFastHLine(0,27,135,TFT_WHITE);
            
            dataSprite.setTextColor(TFT_WHITE,TFT_BLACK);

            if (!config_info.imperial) sprintf(data,"0-%dmeters",(int) config_info.drag.end_distance);
                else sprintf(data,"0-%dfeet",(int) round(config_info.drag.end_distance / FEET_TO_M_FACTOR));

            dataSprite.drawString(F(LANG_DRAG),0,34,4);
            dataSprite.drawRightString(data,135,42,2);
            //width = dataSprite.drawNumber(GPS_info.speed.kmph,0,68,6);
            dataSprite.drawFastHLine(0,120,135,TFT_WHITE);

            if (!config_info.imperial) sprintf(data,"0-%dkmph",(int) config_info.acceleration.end_speed);
                else sprintf(data,"0-%dmph",(int) round(config_info.acceleration.end_speed / M_TO_KM_FACTOR));

            dataSprite.drawString(F(LANG_ACC2),0,127,4);
            dataSprite.drawRightString(data,135,135,2);
            //width = dataSprite.drawNumber(GPS_info.stats.max_kmph,0,161,6);
            dataSprite.drawFastHLine(0,210,135,TFT_WHITE);

            double scoretime = (double) (GPS_info.drag.end_time_alt - GPS_info.drag.start_time_alt) / 1000.0;

            int textsize;

            textsize = (scoretime >= 100.0) ? 4 : 6; 

            if (GPS_info.drag.end_time_alt == 0) 
            {
                strcpy(data,"--");
                textsize = 6;
            }
                else sprintf(data, "%.2f" ,(double) scoretime);

            dataSprite.drawRightString(data,135,68,textsize);

            scoretime = (double) (GPS_info.acceleration.end_time_alt - GPS_info.acceleration.start_time_alt) / 1000.0;

            textsize = (scoretime >= 100.0) ? 4 : 6; 

            if (GPS_info.acceleration.end_time_alt == 0)
            {
                strcpy(data,"--");
                textsize = 6;
            }
                else sprintf(data, "%.2f" ,(double) scoretime);

            dataSprite.drawRightString(data,135,161,textsize);
    } else if (((millis() - 1000) >= drag_count_time) && (drag_countdown > -3))
        {
            drag_countdown--;
            drag_count_time = millis();
            if (drag_countdown < 1) 
            {
                GPS_info.drag.start_time_alt = drag_count_time;
                GPS_info.acceleration.start_time_alt = drag_count_time;
                dataSprite.drawCentreString(F(LANG_GO),67,33,4);
                dataSprite.drawCentreString(F(LANG_GO),67,92,4);
                dataSprite.drawCentreString(F(LANG_GO),67,151,4);
            } else if (drag_countdown > 0)
            {
                sprintf(data,"%d",drag_countdown);
                dataSprite.drawCentreString(data,67,82,8);
            }
            
        }
}

void updateScreen(int refresh_period){
    int width;
    u_int32_t time_screen = millis();

    if (screen_off) return;

    if (time_screen > last_refresh + refresh_period) {
        dataSprite.fillSprite(TFT_BLACK);

    refreshSat();
    refreshBat();
    satSprite.pushToSprite(&dataSprite,0,210,TFT_BLACK);
    batSprite.pushToSprite(&dataSprite,95,210,TFT_BLACK);

    dataSprite.setTextColor(TFT_WHITE,TFT_BLACK);

    switch (screen_id) {
        case 0:
            screen_settings();
        break;
        case 1:
            screen_speed();
        break;
        case 2:
            screen_altitude();
        break;
        case 3:
            screen_distance();
        break;
        case 4:
            screen_location();
        break;
        case 5:
            screen_gforce();
        break;
        case 6:
            screen_scores();
        break;
        case 7:
            screen_drag();
        break;
    }

  dataSprite.pushSprite(0,0);
  last_refresh = time_screen;
    }
    if (last_refresh > time_screen) last_refresh = time_screen;
    if ((config_info.screen_off_time !=0) && ((last_button_press + (config_info.screen_off_time * 1000)) < time_screen)) screenoff();
    if (last_button_press > time_screen) last_button_press = time_screen;

}

void screenoff() {
    screen_off = 1;
    int r = digitalRead(TFT_BL);
    digitalWrite(TFT_BL, !r);
}

void screenon() {
    screen_off = 0;
    digitalWrite(TFT_BL, TFT_BACKLIGHT_ON);
}


void setup() {
    #ifdef DEBUG
        Serial.begin(DEBUG_BAUD);
        Serial.println("\n##################################");
        Serial.println(F("ESP32 Information:"));
        Serial.print(F("  ESP Chip Model:    ")); Serial.println(ESP.getChipModel()); 
        Serial.print(F("  CPU Freq:          ")); Serial.print(ESP.getCpuFreqMHz()); Serial.println(F(" MHz"));
        Serial.print(F("  SDK Version:       ")); Serial.println(ESP.getSdkVersion());
        Serial.print(F("  Heap Size:         ")); Serial.print(ESP.getHeapSize()); Serial.println(F(" bytes"));
        Serial.print(F("  Free Heap:         ")); Serial.print(ESP.getFreeHeap()); Serial.println(F(" bytes"));
        Serial.print(F("  Used Heap:         ")); Serial.print(ESP.getHeapSize()-ESP.getFreeHeap()); Serial.println(F(" bytes"));
        Serial.print(F("  Sketch Size:       ")); Serial.print(ESP.getSketchSize()); Serial.println(F(" bytes"));
        Serial.print(F("  Free Sketch Space: ")); Serial.print(ESP.getFreeSketchSpace()); Serial.println(F(" bytes"));
        Serial.println(F(""));
        Serial.printf("Internal Total Heap %d, Internal Used Heap %d, Internal Free Heap %d\n", ESP.getHeapSize(), ESP.getHeapSize()-ESP.getFreeHeap(), ESP.getFreeHeap());
        Serial.printf("SPIRam Total heap %d, SPIRam Free Heap %d\n", ESP.getPsramSize(), ESP.getFreePsram());
        Serial.printf("ChipRevision %d, Cpu Freq %d, SDK Version %s\n", ESP.getChipRevision(), ESP.getCpuFreqMHz(), ESP.getSdkVersion());
        Serial.printf("Flash Size %d, Flash Speed %d\n", ESP.getFlashChipSize(), ESP.getFlashChipSpeed());
        Serial.println("##################################\n");
    #endif
    delay(500);
    wakeup_reason();
    initConfig();
    initScreen();
    initGPS();
}

void loop() {
readGPS();
updateScreen(SCREEN_REFRESH_PERIOD);
buttonA.loop();
buttonB.loop();
check_activity();
}