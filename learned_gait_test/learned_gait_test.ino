
#include "coor.h"
#include "HCPCA9685.h"          /* Include the HCPCA9685 library created by Andrew Davies */
#include "fmath.h"

#define I2CAdd_board1  0x40    /* Default address of the PCA9685 Module */
#define I2CAdd_board2 0x41    /* Default address of the PCA9685 Module */


/* distance sensor */
const int trigPin = 9;
const int echoPin = 10;
long    duration;
long    distance;
long    currDistance;
long    timeSpent;
float   calSpeed;
long    diffDistance = -1;

/* MPU-6050 */
#include "Wire.h"                                       /* This library allows you to communicate with I2C devices. */
const int MPU_ADDR = 0x68;                        /* I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69. */
int16_t   accelerometer_x, accelerometer_y;       /* variables for accelerometer raw data */

/* Define Leg Numbers */
#define RF  0
#define RM  2
#define RM1 4
#define RE  6
#define LE  7
#define LM1 5
#define LM  3
#define LF  1

HCPCA9685 board1  = HCPCA9685( I2CAdd_board1 );   /* Define Library to use I2C communication */
HCPCA9685 board2  = HCPCA9685( I2CAdd_board2 );   /* Define Library to use I2C communication */


int servoOffset[24] = { 120, 340, 220,
      210, 190, 150,
      120, 110, 180,
      230, 310, 130,
      200, 370, 175,
      230, 135, 240,
      220, 370, 300,
      260, 140, 260 };

int angle_convert_parameter[24] = { 115, 120,  -120,
            110, -115, 115,
            115, -115, 115,
            115, 130,  -115,
            115, 125,  -110,
            110, -115, 115,
            115, 115,  -115,
            115, -120, 120 };

/*  */
/* float Angle_Old[24]; */

/* Body */
/* Length of the Coxa, femur and tibia [mm] */
const int L_Coxa  = 33;
const int L_Femur = 35;
const int L_Tibia = 60;

/* measured according to the hole position */
const int HalfHexwidth  = 45;
const int HexLength1  = 70;
const int HexLength2  = 60;

const int FL_square = L_Femur * L_Femur;
const int TL_square = L_Tibia * L_Tibia;
const int FL_TL   = L_Femur * L_Tibia;

/* angle offset of each leg [degree] */
const int AngleOffset[8] = { 40, 320, 90, 270, 90, 270, 140, 220 };

/* Coordinate of each leg */
coor  Hole_Position[8];
coor  Leg_Position[8];          /* Start positions of the legs */

/*************************/
/* gait */
int GaitStep  = 1;
int LegLiftHeight = 20;

int limitStep = 0;
int StepsInGait = 0;    /* Number of steps in gait */
int GaitStartStep[6];       /* Init position of the leg */

/*selected gait mode*/
int gaitmode = 0;
/*for inclined plane test*/
int inclinedMode = 0;

int learnedMode = 0;
int stoplearning  = 0;

coor GaitPos[8];

/*************************/

long  exeTime = 0;
int speed = 2; /* get from input */

/* pid control parameter applied in the inclined plane test */
int Kp;
int Ki;
int Kd;
int serpoint;
int dt;
int a;
int error;
int integ;
int pre_error = 0;
int diff;
int change;

/*************************/

/* inputs */
coor  BodyPos;
coor  BodyRot;
coor  WalkParameter;    /* Global Input for the walking */


/*************************/

int counter   = 0;
int numbers[8]  = { 0, 1, 2, 3, 4, 5, 6, 7 };
int selected_gait[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
int count = 8;

/* Applied to obtain three angles according to the leg position */
void InverseKinematics( coor Rotation, coor feetPosition, coor leg_offset, int angleOffset, float *CoxaAngle, float *FemurAngle, float *TibiaAngle )
{
  coor leg_position;
  leg_position.X  = feetPosition.X + leg_offset.X;
  leg_position.Z  = feetPosition.Z + leg_offset.Z;
  leg_position.Y  = feetPosition.Y;


  /*
   * T = theta, A = alpha, B = beta
   * calculate according to the table
   */
  float sinT  = fsin( Rotation.X );
  float cosT  = fcos( Rotation.X );
  float sinA  = fsin( Rotation.Z );
  float cosA  = fcos( Rotation.Z );
  float sinB  = fsin( Rotation.Y );
  float cosB  = fcos( Rotation.Y );


  /* Calculate position corrections of feet using Rotation Matrix */
  coor BodyIKPos;

  BodyIKPos.X = (leg_position.X * cosA * cosB - leg_position.Z * cosA * sinB + leg_position.Y * sinA) - leg_position.X;
  BodyIKPos.Z = (leg_position.X * (sinT * sinA * cosB + cosT * sinB) + leg_position.Z * (sinT * sinA * sinB + cosT * cosB) - leg_position.Y * sinT * cosA) - leg_position.Z;
  BodyIKPos.Y = (leg_position.X * (sinT * sinB - cosT * sinA * cosB) + leg_position.Z * (cosT * sinA * sinB + sinT * cosB) + leg_position.Y * cosT * cosA) - leg_position.Y;


  /* update feet position */
  feetPosition.X  = feetPosition.X + BodyIKPos.X;
  feetPosition.Z  = feetPosition.Z + BodyIKPos.Z;
  feetPosition.Y  = feetPosition.Y + BodyIKPos.Y;

  /* calculate according to the angleoffset of each leg*/
  coor RC_FeetPos;

  RC_FeetPos.X  = feetPosition.X * fcos( angleOffset ) - feetPosition.Z * fsin( angleOffset );
  RC_FeetPos.Z  = feetPosition.X * fsin( angleOffset ) + feetPosition.Z * fcos( angleOffset );
  RC_FeetPos.Y  = feetPosition.Y;

  /* Length between the Coxa and Feet */
  float CoxaFeetDist = sqrt( RC_FeetPos.X * RC_FeetPos.X + RC_FeetPos.Z * RC_FeetPos.Z );

  /* IKSW - Length between shoulder and wrist */
  float IKSW = sqrt( (CoxaFeetDist - L_Coxa) * (CoxaFeetDist - L_Coxa) + RC_FeetPos.Y * RC_FeetPos.Y );

  float IKA1  = fatan2( CoxaFeetDist - L_Coxa, RC_FeetPos.Y );
  float IKA2  = facos( (FL_square + IKSW * IKSW - TL_square) / (2.0 * L_Femur * IKSW) );

  /* InverseKinematics FemurAngle */
  *FemurAngle = (IKA1 + IKA2) - PIby2;

  /* InverseKinematics TibiaAngle */
  float TAngle = facos( (TL_square + FL_square - IKSW * IKSW) / (2.0 * FL_TL) );

  /* convert Tibia angle with relation to the normal of Femur axis */
  *TibiaAngle = PIby2 - TAngle;

  /* InverseKinematics CoxaAngle */
  *CoxaAngle = fatan2( RC_FeetPos.X, RC_FeetPos.Z );
}


void GaitSet()
{
  GaitStartStep[RF] = 13;
  GaitStartStep[RM] = 4;
  GaitStartStep[RM1]  = 19;
  GaitStartStep[RE] = 10;
  GaitStartStep[LE] = 22;
  GaitStartStep[LM1]  = 7;
  GaitStartStep[LM] = 16;
  GaitStartStep[LF] = 1;

  limitStep = 27;
  StepsInGait = 21;
}


coor GaitCalculation( coor WalkParameter )
{
  /* Calculate Gait positions */

  for ( int LegIndex = 0; LegIndex < 8; LegIndex++ )
  {
    bool walking = ( (abs( WalkParameter.X ) > 10) || (abs( WalkParameter.Z ) > 10) );
    if ( walking && (GaitStep == GaitStartStep[LegIndex]) )
    {
      GaitPos[LegIndex].Z = -WalkParameter.Z / 2 * FSIN_TABLE[80];
      GaitPos[LegIndex].Y = LegLiftHeight * FSIN_TABLE[20];
    } else if ( walking && (GaitStep == GaitStartStep[LegIndex] + 1) )
    {
      GaitPos[LegIndex].Z = -WalkParameter.Z / 2 * FSIN_TABLE[30];
      GaitPos[LegIndex].Y = LegLiftHeight * FSIN_TABLE[70];
    } else if ( walking && (GaitStep == GaitStartStep[LegIndex] + 2) )
    {
      GaitPos[LegIndex].Z = 0;
      GaitPos[LegIndex].Y = LegLiftHeight;
    } else if ( walking && (GaitStep == GaitStartStep[LegIndex] + 3) )
    {
      GaitPos[LegIndex].Z = WalkParameter.Z / 2 * FSIN_TABLE[30];
      GaitPos[LegIndex].Y = LegLiftHeight * FSIN_TABLE[70];
    } else if ( walking && (GaitStep == GaitStartStep[LegIndex] + 4) )
    {
      GaitPos[LegIndex].Z = WalkParameter.Z / 2 * FSIN_TABLE[80];
      GaitPos[LegIndex].Y = LegLiftHeight * FSIN_TABLE[20];
    } else if ( walking && (GaitStep == GaitStartStep[LegIndex] + 5) )
    {
      GaitPos[LegIndex].Z = WalkParameter.Z / 2;
      GaitPos[LegIndex].Y = 0;
    }
    /* move leg towards*/
    else{
      GaitPos[LegIndex].Z = GaitPos[LegIndex].Z - (WalkParameter.Z / StepsInGait);
      GaitPos[LegIndex].Y = 0;
    }
  }
  /* Advance to the next step */
  if ( ++GaitStep > limitStep )
  {
    GaitStep = 1;
  }
}


long Measuredistance()
{
  /* Clears the trigPin */
  digitalWrite( trigPin, LOW );
  delayMicroseconds( 2 );
/* Sets the trigPin on HIGH state for 10 micro seconds */
  digitalWrite( trigPin, HIGH );
  delayMicroseconds( 10 );
  digitalWrite( trigPin, LOW );
/* Reads the echoPin, returns the sound wave travel time in microseconds */
  duration = pulseIn( echoPin, HIGH );
/* Calculating the distance */
  distance = constrain( duration * 0.034 / 2, 0, 90 );

  return(distance);
}


void resetLeg()
{
  BodyRot.X = 0;
  BodyRot.Y = 0;
  BodyRot.Z = 0;
  BodyPos.X = 0;
  BodyPos.Z = 0;
  BodyPos.Y = 0;
  WalkParameter.X = 0;
  WalkParameter.Y = 0;
  WalkParameter.Z = 0;

  for ( int kk = 0; kk < 8; kk++ )
  {
    GaitPos[kk].X = 0;
    GaitPos[kk].Z = 0;
    GaitPos[kk].Y = 0;
  }
}


void scrambleArray( int * array, int size )
{
  int last  = 0;
  int temp  = array[last];
  for ( int i = 0; i < size; i++ )
  {
    int index = random( size );
    array[last] = array[index];
    last    = index;
  }
  array[last] = temp;
}


/************/

void setup()
{
  /*
   * ==================== Init Values ======================
   * [DIMENSION PARAMETERS]
   */
  Hole_Position[RF].X = HalfHexwidth;                                 /* Distance X from centre of the body to the Right Front coxa */
  Hole_Position[RF].Z = HexLength1 + HexLength2;                      /* Distance Z from centre of the body to the Right Front coxa */
  Hole_Position[RM].X = HalfHexwidth;                                 /* Distance X from centre of the body to the Right Middle coxa */
  Hole_Position[RM].Z = HexLength2;                                   /* Distance Z from centre of the body to the Right Middle coxa */
  Hole_Position[RM1].X  = HalfHexwidth;                                 /* Distance X from centre of the body to the Right Middle1 coxa */
  Hole_Position[RM1].Z  = -HexLength2;                                  /* Distance Z from centre of the body to the Right Middle1 coxa */
  Hole_Position[RE].X = HalfHexwidth;                                 /* Distance X from centre of the body to the Right end coxa */
  Hole_Position[RE].Z = -HexLength1 - HexLength2;                     /* Distance Z from centre of the body to the Right end coxa */


  Hole_Position[LF].X = -HalfHexwidth;                                /* Distance X from centre of the body to the Left Front coxa */
  Hole_Position[LF].Z = HexLength1 + HexLength2;                      /* Distance Z from centre of the body to the Left Front coxa */
  Hole_Position[LM].X = -HalfHexwidth;                                /* Distance Z from centre of the body to the Left Middle coxa */
  Hole_Position[LM].Z = HexLength2;                                   /* Distance Z from centre of the body to the Left Middle coxa */
  Hole_Position[LM1].X  = -HalfHexwidth;                                /* Distance Z from centre of the body to the Left Middle coxa */
  Hole_Position[LM1].Z  = -HexLength2;                                  /* Distance Z from centre of the body to the Left Middle coxa */
  Hole_Position[LE].X = -HalfHexwidth;                                /* Distance X from centre of the body to the Left end coxa */
  Hole_Position[LE].Z = -HexLength1 - HexLength2;                     /* Distance Z from centre of the body to the Left end coxa */


  /*
   * --------------------------------------------------------------------
   * [INITIAL LEG POSITIONS]
   */

  Leg_Position[RF].X  = cos( DegToRad( 50 ) ) * (L_Femur + L_Coxa);   /* Start positions of the Right Front leg */
  Leg_Position[RF].Y  = L_Tibia;
  Leg_Position[RF].Z  = sin( DegToRad( 50 ) ) * (L_Femur + L_Coxa);

  Leg_Position[RM].X  = L_Femur + L_Coxa;                             /* Start positions of the Right Middle leg */
  Leg_Position[RM].Y  = L_Tibia;
  Leg_Position[RM].Z  = 0.0;

  Leg_Position[RM1].X = L_Femur + L_Coxa;                             /* Start positions of the Right Middle1 leg */
  Leg_Position[RM1].Y = L_Tibia;
  Leg_Position[RM1].Z = 0.0;

  Leg_Position[RE].X  = cos( DegToRad( 50 ) ) * (L_Femur + L_Coxa);   /* Start positions of the Right Rear leg */
  Leg_Position[RE].Y  = L_Tibia;
  Leg_Position[RE].Z  = -sin( DegToRad( 50 ) ) * (L_Femur + L_Coxa);

  Leg_Position[LF].X  = -cos( DegToRad( 50 ) ) * (L_Femur + L_Coxa);  /* Start positions of the Left Front leg */
  Leg_Position[LF].Y  = L_Tibia;
  Leg_Position[LF].Z  = sin( DegToRad( 50 ) ) * (L_Femur + L_Coxa);

  Leg_Position[LM].X  = -(L_Femur + L_Coxa);                          /* Start positions of the Left Middle leg */
  Leg_Position[LM].Y  = L_Tibia;
  Leg_Position[LM].Z  = 0.0;

  Leg_Position[LM1].X = -(L_Femur + L_Coxa);                          /* Start positions of the Left Middle1 leg */
  Leg_Position[LM1].Y = L_Tibia;
  Leg_Position[LM1].Z = 0.0;

  Leg_Position[LE].X  = -cos( DegToRad( 50 ) ) * (L_Femur + L_Coxa);  /* Start positions of the Left Rear leg */
  Leg_Position[LE].Y  = L_Tibia;
  Leg_Position[LE].Z  = -sin( DegToRad( 50 ) ) * (L_Femur + L_Coxa);

  Serial.begin( 9600 );

  pinMode( trigPin, OUTPUT );                                             /* Sets the trigPin as an Output */
  pinMode( echoPin, INPUT );                                              /* Sets the echoPin as an Input */


  board1.Init( SERVO_MODE );                                              /* Set to Servo Mode */
  board1.Sleep( false );                                                  /* Wake up PCA9685 module */

  board2.Init( SERVO_MODE );                                              /* Set to Servo Mode */
  board2.Sleep( false );                                                  /* Wake up PCA9685 module */

  Wire.begin();
  Wire.beginTransmission( MPU_ADDR );                                     /* Begins a transmission to the I2C slave (GY-521 board) */
  Wire.write( 0x6B );                                                     /* PWR_MGMT_1 register */
  Wire.write( 0 );                                                        /* set to zero (wakes up the MPU-6050) */
  Wire.endTransmission( true );

  /* ================  Servos  ================== */
  for ( int i = 0; i < 24; i++ )
  {
    if ( i < 12 )
    {
      board1.Servo( i, servoOffset[i] );
    }else {
      board2.Servo( i - 8, servoOffset[i] );
    }
    delay( 5 );
  }

  /*  // ================  Gait  ================== */
  GaitSet();

  delay( 2000 );

  resetLeg();


/* set combinations of differnet groups */
  scrambleArray( numbers, count );
  while ( ( (numbers[0] < 4) && (numbers[1] < 4) && (numbers[2] < 4) ) || ( (numbers[0] > 3) && (numbers[1] > 3) && (numbers[2] > 3) )
    || ( (numbers[3] < 4) && (numbers[4] < 4) && (numbers[5] < 4) ) || ( (numbers[3] > 3) && (numbers[4] > 3) && (numbers[5] > 3) ) )
  {
    scrambleArray( numbers, count );
  }

  currDistance  = Measuredistance();
  exeTime   = millis();
}


void loop()
{
  int i;
  float Angle[24];
  float IKCoxaAngle, IKFemurAngle, IKTibiaAngle;
  coor  posIO;
  coor  rotIO;


  if ( learnedMode == 0 ) /* try one */
  {
/*  Serial.println( Measuredistance() ); */

    WalkParameter.Z = 40;


/* apply the gait */
    GaitStartStep[numbers[0]] = 1;
    GaitStartStep[numbers[1]] = 1;
    GaitStartStep[numbers[2]] = 1;
    GaitStartStep[numbers[3]] = 7;
    GaitStartStep[numbers[4]] = 7;
    GaitStartStep[numbers[5]] = 7;
    GaitStartStep[numbers[6]] = 13;
    GaitStartStep[numbers[7]] = 13;

    limitStep = 18;
    StepsInGait = 12;

/* move towards for 10s */
    if ( millis() - exeTime > 10000 )
    {
      delay( 1000 );
      counter++;
      if ( abs( Measuredistance() - currDistance ) > diffDistance )
      {       /* record gait */
        diffDistance = abs( Measuredistance() - currDistance );
        for ( int i = 0; i < count; i++ )
        /* recorded the optimal gait */
        {
          selected_gait[i] = numbers[i];
        }
        Serial.println( "new:" );
        Serial.println( Measuredistance() - currDistance );
      }

      /* backwards */
      learnedMode = 1;
      exeTime   = millis();
    }

    if ( counter > 1000 )
      learnedMode = 2;
  }else if ( learnedMode == 1 ) /* back wards */
  {
/* move backwards and try new gait */
    GaitSet();
    WalkParameter.Z = -40;
    if ( millis() - exeTime > 13000 )
    {
      delay( 1000 );
      scrambleArray( numbers, count );
      while ( ( (numbers[0] < 4) && (numbers[1] < 4) && (numbers[2] < 4) ) || ( (numbers[0] > 3) && (numbers[1] > 3) && (numbers[2] > 3) )
        || ( (numbers[3] < 4) && (numbers[4] < 4) && (numbers[5] < 4) ) || ( (numbers[3] > 3) && (numbers[4] > 3) && (numbers[5] > 3) ) )
      {
        scrambleArray( numbers, count );
      }
      exeTime   = millis();
      learnedMode = 0;
      currDistance  = Measuredistance();
    }
  }else if ( learnedMode == 2 ) /* finish learning */
  {
    WalkParameter.Z     = 40;
    GaitStartStep[selected_gait[0]] = 1;
    GaitStartStep[selected_gait[1]] = 1;
    GaitStartStep[selected_gait[2]] = 1;
    GaitStartStep[selected_gait[3]] = 7;
    GaitStartStep[selected_gait[4]] = 7;
    GaitStartStep[selected_gait[5]] = 7;
    GaitStartStep[selected_gait[6]] = 13;
    GaitStartStep[selected_gait[7]] = 13;

    limitStep = 18;
    StepsInGait = 12;

    Serial.println( "The learned gait:" );
    for ( int i = 0; i < count; i++ )
    {
      Serial.println( selected_gait[i] );
    }
    Serial.println( "----" );
  }

  /* -------- Gaits Calculation -------------- */
  GaitCalculation( WalkParameter );

  /* // ----- IK ------- */
  for ( int LegIndex = 0; LegIndex < 8; LegIndex++ )
  {
    posIO.X = BodyPos.X + Leg_Position[LegIndex].X + GaitPos[LegIndex].X;
    posIO.Z = BodyPos.Z + Leg_Position[LegIndex].Z + GaitPos[LegIndex].Z;
    posIO.Y = BodyPos.Y + Leg_Position[LegIndex].Y + GaitPos[LegIndex].Y;

    rotIO.X = BodyRot.X;
    rotIO.Z = BodyRot.Z;
    rotIO.Y = BodyRot.Y;

    InverseKinematics( rotIO, posIO, Hole_Position[LegIndex], AngleOffset[LegIndex], &IKCoxaAngle, &IKFemurAngle, &IKTibiaAngle );

    Angle[i++]  = IKCoxaAngle;
    Angle[i++]  = IKFemurAngle;
    Angle[i++]  = IKTibiaAngle;
  }


  /* PrintServoAngles(); */
  for ( i = 23; i >= 0; i-- )
  {
    if ( i < 12 )
    {
      board1.Servo( i, servoOffset[i] + angle_convert_parameter[i] * Angle[i] );
    } else{
      board2.Servo( i - 8, servoOffset[i] + angle_convert_parameter[i] * Angle[i] );
    }
    delay( 10 );
  }
}
