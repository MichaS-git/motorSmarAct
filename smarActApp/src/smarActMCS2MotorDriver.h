/*
FILENAME...   SmarActMCS2MotorDriver.h
USAGE...      Motor driver support for the SmarAct MCS2 controller.

David Vine
Adapted from Mark Rivers' ACR Driver
Jan 19, 2019

Note:
The MCS2 controller uses 64-bit int for the encoder and target positions. The motor record is limited
to 32 bit int for RMP (https://github.com/epics-modules/motor/issues/8,
https://epics.anl.gov/tech-talk/2018/msg00087.php) which effectively limits the travel
range to +/- 2.1mm.
Since it doesn't seem the motor record will update to using 64bit int the choices I can see are:
* 1 - using a non-standard motor support
* 2 - rescaling the minimum resolution to 1nm to effectively increase the range to 2.1m

I chose option 2.
1 step = 1nm

Someone with more experience may have a better solution.

Note on controller capability:
The controller supports many more sophisticated features than are supported in this driver.
The two that may be of significant interest are:
  * TTL triggering at specified positions
  * "scan" mode where the piezo stick slip can flex up to 1.6micron to give
     very precise and fast motion

*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"

/* This is the same for lin and rot positioners
 * lin: controller pm --> driver nm. Because of this the user can use the positioner for mm ranges
 * rot: controller ndeg --> driver udeg. Because of this the user can use the positioner for deg ranges
 * If this scaling was not implemented the maximum range would be ~2.147 mm/deg, now it's ~2147 mm/deg */
#define PULSES_PER_STEP 1000
#define GET_VARIABLE_NAME(Variable) (#Variable)

/** MCS2 Axis status flags **/
const unsigned short ACTIVELY_MOVING         = 0x0001;
const unsigned short CLOSED_LOOP_ACTIVE      = 0x0002;
const unsigned short CALIBRATING             = 0x0004;
const unsigned short REFERENCING             = 0x0008;
const unsigned short MOVE_DELAYED            = 0x0010;
const unsigned short SENSOR_PRESENT          = 0x0020;
const unsigned short IS_CALIBRATED           = 0x0040;
const unsigned short IS_REFERENCED           = 0x0080;
const unsigned short END_STOP_REACHED        = 0x0100;
const unsigned short RANGE_LIMIT_REACHED     = 0x0200;
const unsigned short FOLLOWING_LIMIT_REACHED = 0x0400;
const unsigned short MOVEMENT_FAILED         = 0x0800;
const unsigned short STREAMING               = 0x1000;
const unsigned short OVERTEMP                = 0x4000;
const unsigned short REFERENCE_MARK          = 0x8000;

/** MCS2 Axis reference options **/
const unsigned short   START_DIRECTION         = 0x0001;
const unsigned short   REVERSE_DIRECTION       = 0x0002;
const unsigned short   AUTO_ZERO               = 0x0004;
const unsigned short   ABORT_ON_END_STOP       = 0x0008;
const unsigned short   CONTINUE_ON_REF_FOUND   = 0x0010;
const unsigned short   STOP_ON_REF_FOUND       = 0x0020;

/** MCS2 Axis constants **/
#define HOLD_FOREVER 0xffffffff

/** drvInfo strings for extra parameters that the MCS2 controller supports */
#define MCS2MclfString "MCLF"
#define MCS2PtypString "PTYP"
#define MCS2PtypRbString "PTYP_RB"
#define MCS2PstatString "PSTAT"
#define MCS2RefString "REF"
#define MCS2CalString "CAL"

/** I/O-Module parameters**/
#define MCS2IoVoltString "IOVOLT"
#define MCS2IoEnableString "IOENABLE"
/** Channel 1 **/
#define MCS2Ch1TrModeString "CH1_TRMODE"
#define MCS2Ch1TrPolarityString "CH1_TRPOLARITY"
#define MCS2Ch1TrWidthString "CH1_TRWIDTH"
#define MCS2Ch1TrStartString "CH1_TRSTART"
#define MCS2Ch1TrIncrementString "CH1_TRINCREMENT"
#define MCS2Ch1TrMinString "CH1_TRMIN"
#define MCS2Ch1TrMaxString "CH1_TRMAX"
#define MCS2Ch1TrDirectionString "CH1_TRDIRECTION"
/** Channel 2 **/
#define MCS2Ch2TrModeString "CH2_TRMODE"
#define MCS2Ch2TrPolarityString "CH2_TRPOLARITY"
#define MCS2Ch2TrWidthString "CH2_TRWIDTH"
#define MCS2Ch2TrStartString "CH2_TRSTART"
#define MCS2Ch2TrIncrementString "CH2_TRINCREMENT"
#define MCS2Ch2TrMinString "CH2_TRMIN"
#define MCS2Ch2TrMaxString "CH2_TRMAX"
#define MCS2Ch2TrDirectionString "CH2_TRDIRECTION"
/** Channel 3 **/
#define MCS2Ch3TrModeString "CH3_TRMODE"
#define MCS2Ch3TrPolarityString "CH3_TRPOLARITY"
#define MCS2Ch3TrWidthString "CH3_TRWIDTH"
#define MCS2Ch3TrStartString "CH3_TRSTART"
#define MCS2Ch3TrIncrementString "CH3_TRINCREMENT"
#define MCS2Ch3TrMinString "CH3_TRMIN"
#define MCS2Ch3TrMaxString "CH3_TRMAX"
#define MCS2Ch3TrDirectionString "CH3_TRDIRECTION"

class epicsShareClass MCS2Axis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  MCS2Axis(class MCS2Controller *pC, int axis);
  void report(FILE *fp, int level);
  asynStatus poll(bool *moving);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus setPosition(double position);

private:
  MCS2Controller *pC_;      /**< Pointer to the asynMotorController to which this axis belongs.
                                *   Abbreviated because it is used very frequently */
  int channel_;
  asynStatus comStatus_;


friend class MCS2Controller;
};

class epicsShareClass MCS2Controller : public asynMotorController {
public:
  MCS2Controller(const char *portName, const char *MCS2PortName, int numAxes, double movingPollPeriod, double idlePollPeriod, int unusedMask = 0);
  virtual asynStatus clearErrors();

  /* These are the methods that we override from asynMotorDriver */
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);

  /* These are the methods that we override from asynMotorDriver */
  void report(FILE *fp, int level);
  MCS2Axis* getAxis(asynUser *pasynUser);
  MCS2Axis* getAxis(int axisNo);

protected:
  int mclf_; /**< MCL frequency */
#define FIRST_MCS2_PARAM mclf_
  int ptyp_; /**< positioner type */
  int ptyprb_; /**< positioner type readback */
  int pstatrb_; /**< positoner status word readback */
  int ref_;  /**< reference command */
  int cal_;  /**< calibration command */
/** I/O Module options **/
  int iovolt_;
  int ioenable_;
/** channel 1 **/
  int ch1trmode_;
  int ch1trpolarity_;
  int ch1trwidth_;
  int ch1trstart_;
  int ch1trincrement_;
  int ch1trmin_;
  int ch1trmax_;
  int ch1trdirection_;
/** channel 2 **/
  int ch2trmode_;
  int ch2trpolarity_;
  int ch2trwidth_;
  int ch2trstart_;
  int ch2trincrement_;
  int ch2trmin_;
  int ch2trmax_;
  int ch2trdirection_;
/** channel 3 **/
  int ch3trmode_;
  int ch3trpolarity_;
  int ch3trwidth_;
  int ch3trstart_;
  int ch3trincrement_;
  int ch3trmin_;
  int ch3trmax_;
  int ch3trdirection_;
#define LAST_MCS2_PARAM ch3trdirection_
#define NUM_MCS2_PARAMS (&LAST_MCS2_PARAM - &FIRST_MCS2_PARAM + 1)

friend class MCS2Axis;
};

