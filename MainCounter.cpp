
// New Version 

#include <Servo.h>

#define HP_MAXIMUMx10      215
#define HP_RESTRICTIONx10  150

//HW Definition
#define LED_PIN         13
#define INPUT_PIN        7
#define SERVO_PIN        9

//main Timer
#define TIME_INTERVAL  200 //ms

//Servo Definition
#define SERVO_MIN_RESTRICT   70
#define SERVO_MAX_RESTRICT  130
#define SERVO_COEF            1

//Table Elements
#define MAX_ELEMENTS    44

// Pins Define
const byte   InterruptPin = 3;
int          SensorPin    = A0;    // select the input pin for the potentiometer

//Parameters

Servo myservo;

unsigned int HP_RANGE    = HP_MAXIMUMx10-HP_RESTRICTIONx10;
unsigned int SERVO_RANGE = SERVO_MAX_RESTRICT-SERVO_MIN_RESTRICT;

//RPM Parameters
unsigned int          CountsToRpm   =  60 * ( 1000 /TIME_INTERVAL);
volatile unsigned int Counts    = 0;  //  Counter


unsigned int currRpm=0,prevRpm=0;
int currPos=0, prvPos = SERVO_MIN_RESTRICT;
unsigned long prevMillis=0;

unsigned long preISRvMillis=0;
unsigned long currISRvMillis=0;

unsigned int xCounts,rpm;

double sensorValue=0;

unsigned int index ;
unsigned int hp;
int pos;




//Data 
///////////////////////////////////////////////////////////////////////////////////////////////
//RPM Vr. HP Table
int RpmVecEntry =  6000;
int RpmVEcDelta =  200;
int RpmStartRestriction = 7200;
int RpmEndRestriction   = 15200;

//Table
///////////////////////////////////////////////////////////////////////////////////////////////
byte rpmhpTable[MAX_ELEMENTS] =//RPM Vs HP*10
        {
                98,104,109,116,122,129,138,140,150,155,
                149,150,159,173,180,191,196,202,208,215,
                214,214,214,210,215,202,198,193,192,190,
                188,187,184,184,181,177,173,168,168,168,
                167,164,160,150
        };
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
    Serial.begin(19200);

    pinMode(LED_PIN, OUTPUT);   // Pin 13 has an LED connected

    myservo.attach(SERVO_PIN);     // attaches the servo on pin 9 to the servo object

    pinMode(InterruptPin, INPUT_PULLUP);
    attachInterrupt( digitalPinToInterrupt(InterruptPin), isrCounter , FALLING);
    // digitalWrite(InterruptPin,HIGH);

    myservo.write(SERVO_MIN_RESTRICT);

    delay(TIME_INTERVAL);
    // For Operation Version Printout
    // PrintData();
    // Serial.end();  //@@DEBUG
}

void loop()
{
    if ( (millis()-prevMillis) > TIME_INTERVAL )
    {
        prevMillis=millis();
        xCounts = Counts+15;  //@@@@   For Debug I add 15 Counts for Higher RPM
        Counts=0;

        currRpm = (xCounts *CountsToRpm+prevRpm)/2;  //average between two Consecutive RPM
        prevRpm = currRpm;// xCounts *CountsToRpm;

        // Toggel Use XOR 4 Health Inspection
        digitalWrite(LED_PIN, digitalRead( LED_PIN ) ^ 1 );

        //Serial.print("rpm=");
        // Serial.println(currRpm);

        currPos = RpmToServoPos(currRpm);

        //Serial.println(currPos);

        //Activate Servo
        if ( prvPos != currPos )
        {
            myservo.write(currPos);
            prvPos = currPos;
            // Serial.print(rpm);
            // Serial.print("<< RPM-----SERVO MOVE TO >> ");
            // Serial.println(pos);
        }
    }
}



/////////////////////////////////////////////////////////////////////////////
//                     I S R   
////////////////////////////////////////////////////////////////////////////

void isrCounter()
{
    sensorValue = analogRead(SensorPin);
    if ( sensorValue  < 1000 ) return;   //1000 [mv]  Threshold

    // ToDo: Check if needed ??????
    currISRvMillis = millis();
    if (( currISRvMillis-preISRvMillis) <= 2 ) return;
    preISRvMillis =currISRvMillis;
    // Serial.print( "Counts");//millis()-preISRvMillis);

    Counts=Counts+1;  //Counts++ is Not Good By Definition of volatile unsigned int Counts ??
}
///////////////////////////////////////////////////////////////////////////


int RpmToServoPos(unsigned int rpm) //Counts to Rpm
{

    // End Conditions
    if ( rpm  <=     RpmStartRestriction ) return  SERVO_MIN_RESTRICT;
    else if ( rpm  > RpmEndRestriction   ) return  SERVO_MIN_RESTRICT;

    //Finds Table Index
    index = (int)( (double) (rpm- RpmVecEntry)/(double) RpmVEcDelta +0.5);

    // End Conditions After Index Calculation
    if ( index < 0 ) return SERVO_MIN_RESTRICT;                   // No restriction
    else if ( index >= MAX_ELEMENTS ) return SERVO_MIN_RESTRICT;  // No restriction

    // Calculates HP
    hp  = (unsigned int) rpmhpTable[(int) index] ;

    if ( hp <= HP_RESTRICTIONx10)  return  SERVO_MIN_RESTRICT; //Under Restriction trashold
    if ( hp >= HP_MAXIMUMx10)      return  SERVO_MAX_RESTRICT; //Above Max trashold
///////////////////////////////////////////////////////////////////////////////////////////////////

    pos = SERVO_MIN_RESTRICT+  (int) ((double)(SERVO_RANGE *( hp - HP_RESTRICTIONx10) )/(double)HP_RANGE);

    if ( pos < SERVO_MIN_RESTRICT)
    {
        pos = SERVO_MIN_RESTRICT;
    }
    else if ( pos > SERVO_MAX_RESTRICT)
    {
        pos = SERVO_MAX_RESTRICT;
    }
    return pos;
}






///////////////////////////////////////////////////////////////////////////////
//Utilities Functions
//////////////////////////////////////////////////////////////////////////////


void PrintData()
{
    //title
    Serial.print(" DATA PARAMETERS FOR ENGINE XXXX - HP RESTICTION  IS ");
    Serial.println(HP_RESTRICTIONx10/10);
    Serial.println("");
    Serial.println("                      RESTRICTION RANGE");
    int maxRange = RpmVecEntry + (MAX_ELEMENTS-1)* RpmVEcDelta;
    Serial.print("     Minimal RPM ="); Serial.print(RpmVecEntry);Serial.print("      Maximal RPM ="); Serial.println(maxRange);
    Serial.println("");

    Serial.println("    RPM Vs HP Table");

    int rpm = RpmVecEntry;
    for ( int i=0;i< MAX_ELEMENTS;i++ )
    {
        int k = i+1;
        if ( i== 0 )         {Serial.print("   "); Serial.print(k); Serial.print(" |  ");Serial.print(rpm); Serial.print(" | ");Serial.println(rpmhpTable[i]);}
        else if ( k <10 )    {Serial.print("   "); Serial.print(k); Serial.print(" | ");Serial.print(rpm); Serial.print(" | ");Serial.println(rpmhpTable[i]);}
        else                 {Serial.print("  "); Serial.print(k); Serial.print(" | ");Serial.print(rpm); Serial.print(" | ");Serial.println(rpmhpTable[i]);}

        rpm +=RpmVEcDelta;
    }
    Serial.println("---------------------------------------------------------");
    Serial.println("                         End of Data");

}