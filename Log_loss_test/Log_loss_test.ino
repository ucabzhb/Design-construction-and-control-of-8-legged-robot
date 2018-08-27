#include "coor.h"
#include "HCPCA9685.h"          /* Include the HCPCA9685 library created by Andrew Davies */
#include "fmath.h"
#include "Wire.h"

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

/* MPU-6050 */
/* This library allows you to communicate with I2C devices. */
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

/* servo constrains */
int servoMin[24] = { -30, -30, -30 };
int servoMax[24] = { 30, 30, 30 };

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

const int startleglist[8] = { 7, 5, 3, 1, 6, 4, 2, 0 };
const int convert[8] = { 7, 3, 6, 2, 5, 1, 4, 0 };
int   value = 10;

/* Coordinate of each leg */
coor  Hole_Position[8];
coor  Leg_Position[8];          /* Start positions of the legs */

/*************************/
/* gait */
int GaitStep  = 1;
int LegLiftHeight = 20;

int limitStep = 0;
int StepsInGait = 0;    /* Number of steps in gait */
int GaitStartStep[8];       /* Init position of the leg */


/*selected gait mode*/
int gaitmode = 0;
/*for inclined plane test*/
int inclinedMode = 0;

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

int turning_mode  = 0;
int turning_counter = 0;

/*************************/

/* inputs */
coor  BodyPos;
coor  BodyRot;
coor  WalkParameter;    /* Global Input for the walking */


/*************************/

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
  GaitStartStep[RF] = 43;
  GaitStartStep[RM] = 37;
  GaitStartStep[RM1]  = 31;
  GaitStartStep[RE] = 25;
  GaitStartStep[LE] = 1;
  GaitStartStep[LM1]  = 7;
  GaitStartStep[LM] = 13;
  GaitStartStep[LF] = 19;

  limitStep = 48;
  StepsInGait = 42;
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


  board1.Init( SERVO_MODE );                                              /* Set to Servo Mode */
  board1.Sleep( false );                                                  /* Wake up PCA9685 module */

  board2.Init( SERVO_MODE );                                              /* Set to Servo Mode */
  board2.Sleep( false );                                                  /* Wake up PCA9685 module */

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

  Serial.println( "Enter the losing leg number (0-7):" );
}


void loop()
{
  char  recValue;
  int mode;
  int k = 1;
  int i = 0;
  float Angle[24];
  float IKCoxaAngle, IKFemurAngle, IKTibiaAngle;
  coor  posIO;
  coor  rotIO;


  if ( Serial.available() > 0 )
  {
    recValue = Serial.read();
    /* check if a number was received */
    if ( (recValue >= '0') && (recValue <= '9') )
    {
      value = recValue - '0';
    }

    if ( (value >= 0) && (value <= 7) )
    {
      limitStep = 42;
      StepsInGait = 36;

      k = 1;
      for ( int LegIndex = 0; LegIndex < 8; LegIndex++ )
      {
        GaitStartStep[startleglist[LegIndex]] = k;
        if ( LegIndex != convert[value] )
        {
          k = k + 6;
        }else  {
          { GaitStartStep[startleglist[LegIndex]] = 0; }
        }
      }
    }
  } /* end: if (Serial.available() > 0) */


  /* -------- Gaits Calculation -------------- */
  WalkParameter.Z = 40;

  GaitCalculation( WalkParameter );


  BodyPos.X = constrain( BodyPos.X, -30, 30 );
  BodyPos.Z = constrain( BodyPos.Z, -40, 40 );
  BodyRot.Z = constrain( BodyRot.Z, -11, 11 );

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

    if ( LegIndex == value )
    {
      Angle[i++]  = 0;
      Angle[i++]  = -0.9;
      Angle[i++]  = -0.4;
    }else  {
      Angle[i++]  = IKCoxaAngle;
      Angle[i++]  = IKFemurAngle;
      Angle[i++]  = IKTibiaAngle;
    }
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
    delay( 5 );
  }
}



