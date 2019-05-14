/*************************************************************************/
/**************************** CONFIGURE BELOW ****************************/
/*************************************************************************/

/**
 * DEBUG - Enable/Disable for debugging purposes
 */
    uint8_t DEBUG = true;

/**
 * IN MINUTES, how many will need to pass before each gps push
 */
    const uint8_t gps_push = 10;

/*************************************************************************/
/************************* VARIABLES & LIBRARIES *************************/
/*************************************************************************/

/**
 * INLCLUDE LIBRARIES HERE
 */
    #include <TinyGPS++.h>

/**
 * REQUIRED LIBRARY CALLS
 */
    FuelGauge   fuel;
    TinyGPSPlus gps;

/**
 * SETTINGS & MEMORY
 */
    // Particle-firebase
    const char *PUBLISH_EVENT_NAME = "wander-push-firebase-v1";

    // Device
    bool    is_live = false;
    float   battery_life;
    uint8_t push = gps_push;

    // SATELLITES
    bool sat_fix = false;

    // GPS
    const unsigned long MAX_GPS_AGE_MS = 10000; // GPS location must be newer than this to be considered valid
    float start_timer = 0;

    float latitude;
    float longitude;
    float altitude;
    float course;
    float speed;
    float satellites;

/*************************************************************************/
/****************************** SETUP & LOOP *****************************/
/*************************************************************************/

void setup() {
    // Wait for hardware serial to appear
    while (!Serial1);

    /********************* Enable the GPS Module *************************/
    Serial1.begin(9600);
    pinMode(D6, OUTPUT);
    digitalWrite(D6, LOW);

    /********************* Establishing the Timer ************************/
    start_timer = millis();

    /***************************** DEBUG *********************************/
    if( DEBUG && Particle.connected() ) {
        Particle.publish("DEBUG", "Initializing the device...", PRIVATE);
    }
}

/*************************************************************************/

void loop() {

    // We need to check if is live first, this mode will send the cordenates every minutes
    // If is live, this function will set the push variable to send data every minute
    check_live_status();

    // We use the Sat fix variable to keep looping every 60 sec until we get the gps data
    // We also limit the time of every push is the sat is already fix
    if( sat_fix == false && millis() > start_timer + (60000)  || sat_fix == true && millis() > start_timer + (60000*push) ) {

        /********************** Resetting the Timer *************************/
        start_timer = millis();

        /************************* Battery Life *****************************/
        set_batterylife();

        /************************* GET GPS DATA *****************************/
        gps_variables();

        /********************* Send GPS to Firebase *************************/
        if( sat_fix == true ) {
            // Send the variables to firebase
            firebase_push( battery_life, "true", latitude, longitude, satellites );
        }

        else {
            // Let Firebase now that there is no sat Fix at the moment
            firebase_push( battery_life, "false", 0, 0, 0 );
        }
    }

    /********************* ********************  **************************/
    /********************* BATTERY OPTIMIZATION **************************/
    /********************* ******************** **************************/

    battery_optimization();

}

/*************************************************************************/
/******************************** Functions ******************************/
/*************************************************************************/

void check_live_status() {

    // So if the user wants to know the GPS cordenate every second he will activate'
    // th live mode, after that it will change the live mode variable of this DEVICE
    // to true on firebase and at that point the gps_push is going to be changed to
    // 1 min per publish
    bool is_live_var_from_firebase = false;

    if( is_live_var_from_firebase == true ) {
        is_live = true;

        // We set the publish variable to 1 min per push
        push = 1;
    }

    else {
        is_live = false;

        // We set the publish variable to its standar value
        push = gps_push;
    }
}

/*************************************************************************/

void set_batterylife() {
    battery_life = fuel.getSoC();

    /***************************** DEBUG *********************************/
    if( DEBUG && Particle.connected() ) {
        Particle.publish("DEBUG", "Battery left: %" +  String(battery_life), PRIVATE);
    }
}

/*************************************************************************/

void gps_variables() {

    // For the boron to be able to access the data it will need to get it
    // Using the serial channel and add it to the TinyGPSPlus library to manage it
    while( Serial1.available() > 0 ) {
        gps.encode(Serial1.read());
    }

    // To be able to establish the variables, we need to get a read first
    if( gps.location.isValid() ) {

        /**
         * Variables available to send to database
         *    LATITUDE: gps.location.lat() | In degrees (double)
         *    LONGITUDE: gps.location.lng() | In degrees (double)
         *    ALTITUDE: gps.altitude.feet() or .meters() or .miles() or .kilometers() | (double)
         *    COURSE: gps.course.deg() | (double)
         *    SPEED: gps.speed.mph() or .knots() or .mps() or .kmph()  | (double)
         *    SATS: gps.satellites.value()
         *
         *    Elapsed time to get the satelite fix: elapsed
         *    Get accuracy of location: gps.hdop.value() / 10
         *    Direction: TinyGPSPlus::cardinal(gps.course.value())
         *    String to google maps: String(latitude) + "," + String(longitude);
         */

        latitude   = gps.location.lat();
        longitude  = gps.location.lng();
        altitude   = gps.altitude.feet();
        course     = gps.course.deg();
        speed      = gps.speed.mph();
        satellites = gps.satellites.value();

        // We tell the device that we are fix to a sat and have the data
        sat_fix = true;

        /***************************** DEBUG *********************************/
        if( DEBUG && Particle.connected() ) {
            Particle.publish("DEBUG", "GPS SAT FIX...", PRIVATE);
        }
    }

    else {
        // There was no GPS SAT FIX at this moment
        sat_fix = false;

        /***************************** DEBUG *********************************/
        if( DEBUG && Particle.connected() ) {
            Particle.publish("DEBUG", "There is no GPS SAT FIX at this moment...", PRIVATE);
        }
    }
}

/*************************************************************************/

void firebase_push( float battery_life, char* sat_fix, float latitude, float longitude, float satellites ) {
    // Last bracket used to close the json push
    Particle.publish(PUBLISH_EVENT_NAME, "{\"battery_life\":\"" +  String(battery_life) + "\", \"gps_data\": { \"sat_fix\": \"" +  String(sat_fix) + "\", \"latitude\": \"" +  String(latitude) + "\", \"longitude\": \"" +  String(longitude) + "\", \"satellites\": \"" +  String(satellites) + "\" } }", PRIVATE);
}

/*************************************************************************/

void battery_optimization() {

  // Right now there is no way to save energy on a boron
  // Particle has not released the sleep modes
  // You will spend more energy turn on/off antenna/GPS
    //https://community.particle.io/t/current-consumption-and-sleep-modes-boron/45612/64
    //https://community.particle.io/t/boron-minimizing-battery-and-data-usage/46438

}

/*************************************************************************/
/*************************************************************************/
/*************************************************************************/
