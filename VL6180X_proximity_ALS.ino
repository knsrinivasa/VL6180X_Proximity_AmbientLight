/* This example demonstrates how to use interleaved mode to
  take continuous range and ambient light measurements. The
  datasheet recommends using interleaved mode instead of
  running "range and ALS continuous modes simultaneously (i.e.
  asynchronously)".

  In order to attain a faster update rate (10 Hz), the max
  convergence time for ranging and integration time for
  ambient light measurement are reduced from the normally
  recommended defaults. See section 2.4.4 ("Continuous mode
  limits") and Table 6 ("Interleaved mode limits (10 Hz
  operation)") in the VL6180X datasheet for more details.

  Raw ambient light readings can be converted to units of lux
  using the equation in datasheet section 2.13.4 ("ALS count
  to lux conversion").

  Example: A VL6180X gives an ambient light reading of 613
  with the default gain of 1 and an integration period of
  50 ms as configured in this sketch (reduced from 100 ms as
  set by configureDefault()). With the factory calibrated
  resolution of 0.32 lux/count, the light level is therefore
  (0.32 * 613 * 100) / (1 * 50) or 392 lux.

  The range readings are in units of mm. */

// ----------  Includes --------------------------
#include <Wire.h>
#include <VL6180X.h>
#include "Adafruit_VL6180X.h"

Adafruit_VL6180X vl = Adafruit_VL6180X();
// ------------ Defines ------------------------

#define FALSE    0
#define TRUE     (!FALSE)

//#define DARK_THRESHOLD_LX         50
//#define PROXIMITY_DEB_CNT         20

#define PROXIMITY_THRESHOLD_MM    40

#define PROXIMITY_WAKEUP_TIME_MS     2000
#define PROXIMITY_SLEEP_TIME_MS      4000
#define PROXIMITY_SHUTDOWN_TIME_MS   7000


typedef enum System_ProxStates_e
{
  PROXIMITY_INIT,
  PROXIMITY_WAKEUP,
  PROXIMTY_SLEEP,
  PROXIMITY_SHUTDOWN,
} System_ProxStates_e ;


/// Data associated with state machine
typedef struct Proximity_SM_stateObj
{
  System_ProxStates_e currState;      // Current state of Proximity_SM
  System_ProxStates_e nextState;       // Next state of Proximity_SM
} Proximity_SM_stateObj;

// state machine data structure
static Proximity_SM_stateObj st_Proximity_SM_States = {PROXIMITY_INIT} ;

// ------------ Prototypes ------------------------
void Set_IsNear_st(bool b_Proximity_st) ;

// constants won't change. They're used here to
// set pin numbers:
const int PROCESSOR_PIN = 7;       // the number of the processor trigger pin
const int PROJECTOR_PIN = 8;      // the number of the projector status pin


//static System_ProxStates_e e_uC_Init_flag = PROXIMITY_WAIT ;  // Flag to indiate system initialized

VL6180X sensor;

/**************************************************************************************************************************************/

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  sensor.init();
  sensor.configureDefault();

  // Reduce range max convergence time and ALS integration
  // time to 30 ms and 50 ms, respectively, to allow 10 Hz
  // operation (as suggested by Table 6 ("Interleaved mode
  // limits (10 Hz operation)") in the datasheet).
  sensor.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  sensor.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);

  sensor.setTimeout(500);

  // stop continuous mode if already active
  sensor.stopContinuous();
  // in case stopContinuous() triggered a single-shot
  // measurement, wait for it to complete
  delay(300);
  // start interleaved continuous mode with period of 100 ms
  sensor.startInterleavedContinuous(100);


  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  // set the modes for the pins
  pinMode(PROCESSOR_PIN, OUTPUT);
  pinMode(PROJECTOR_PIN, OUTPUT);

}


/**************************************************************************************************************************************/

unsigned long Get_ProximityDebDur_ms(bool b_proxim_st)
{
  static bool b_StartDeb_flag                      = FALSE ;  // Proximity debounce start flag
  static unsigned long t_DebounceStartTime_ms      = 0  ;     // the time the debounce for proximity was started

  // evaluate proximity if object is close for more than defined rasters
  if ( !b_StartDeb_flag && !b_proxim_st )
    t_DebounceStartTime_ms = millis()  ;
  else if ( !b_StartDeb_flag && b_proxim_st )
  {
    t_DebounceStartTime_ms = millis()  ;
    b_StartDeb_flag = TRUE ;
  }
  else if ( b_proxim_st )
  {
    // do debounce
  }
  else
  {
    b_StartDeb_flag = FALSE ;
    return ( millis() - t_DebounceStartTime_ms ) ;
  }

  return 0 ;
}

/**************************************************************************************************************************************/
/*
  Set_ControlPins( System_ProxStates_e e_set_mode)
  {
  static unsigned long t_PinCtrlTime_ms      = 0  ;     // the time the debounce for proximity was started
  static bool b_tmrStartFlg = FALSE ;
  static bool b_Pulse_gen_flag = FALSE ;

  switch ()
  {
    case PROXIMITY_INIT:
      {
        digitalWrite(PROCESSOR_PIN, LOW);   // send a pulse on Processor trigger Pin for 500 ms
        digitalWrite(PROJECTOR_PIN, LOW);   // turn the Projector Pin OFF (LOW is the voltage level)
        break ;
      }

    case PROXIMITY_WAKEUP:
      {
        if ( !b_tmrStartFlg && !b_WakePulse_gen_flag)
        {
          digitalWrite(PROCESSOR_PIN, HIGH);   // send a pulse on Processor trigger Pin for 500 ms
          t_PinCtrlTime_ms = millis() ;
          b_tmrStartFlg = TRUE ;
          b_WakePulse_gen_flag = TRUE ;
        }
        else
        {
          if ( ( millis() - t_PinCtrlTime_ms ) >= PROCCESOR_TRIG_WIDTH_MS )
          {
            digitalWrite(PROCESSOR_PIN, LOW);   // send a pulse on Processor trigger Pin for 500 ms
            b_tmrStartFlg = FALSE ;
          }
        }

        digitalWrite(PROJECTOR_PIN, HIGH);   // turn the Projector Pin ON (HIGH is the voltage level)
        break ;
      }

    case PROXIMTY_SLEEP:
      {
        if ( !b_tmrStartFlg && !b_SleepPulse_gen_flag)
        {
          digitalWrite(PROCESSOR_PIN, HIGH);   // send a pulse on Processor trigger Pin for 500 ms
          t_PinCtrlTime_ms = millis() ;
          b_tmrStartFlg = TRUE ;
          b_SleepPulse_gen_flag = TRUE ;
        }
        else
        {
          if ( ( millis() - t_PinCtrlTime_ms ) >= PROCCESOR_TRIG_WIDTH_MS )
          {
            digitalWrite(PROCESSOR_PIN, LOW);   // send a pulse on Processor trigger Pin for 500 ms
            b_tmrStartFlg = FALSE ;
          }
        }

        digitalWrite(PROJECTOR_PIN, LOW);   // turn the Projector Pin OFF (LOW is the voltage level)
        break ;
      }

    case PROXIMITY_SHUTDOWN:
      {
        digitalWrite(PROCESSOR_PIN, LOW);   // send a pulse on Processor trigger Pin for 500 ms
        digitalWrite(PROJECTOR_PIN, LOW);   // turn the Projector Pin OFF (LOW is the voltage level)
        break ;
      }

  case default:
      {
        digitalWrite(PROCESSOR_PIN, LOW);   // send a pulse on Processor trigger Pin for 500 ms
        digitalWrite(PROJECTOR_PIN, LOW);   // turn the Projector Pin OFF (LOW is the voltage level)
        break ;
      }

  }

  }
*/
/**************************************************************************************************************************************/

void Set_IsNear_st(bool b_Proximity_st )
{
  static bool toggle_st = 0 ;
  unsigned long t_ProximityDebDur_ms = 0  ;     // total proximity duration

  t_ProximityDebDur_ms = Get_ProximityDebDur_ms(b_Proximity_st) ;

  if (st_Proximity_SM_States.nextState != PROXIMITY_INIT)
    st_Proximity_SM_States.currState = st_Proximity_SM_States.nextState ;

  switch (st_Proximity_SM_States.currState)
  {
    case PROXIMITY_INIT:
      {
        if (t_ProximityDebDur_ms >= PROXIMITY_WAKEUP_TIME_MS)
        {
          Serial.print("\n\t\t\t--- PROXIMITY Wake-up---\n");

          digitalWrite(PROCESSOR_PIN, HIGH);   // send a pulse on Processor trigger Pin for 500 ms
          delay(500);
          digitalWrite(PROCESSOR_PIN, LOW);   // send a pulse on Processor trigger Pin for 500 ms

          digitalWrite(PROJECTOR_PIN, HIGH);   // turn the Projector Pin on (HIGH is the voltage level)

          st_Proximity_SM_States.nextState = PROXIMITY_WAKEUP ;              // Flag to indiate system initialized

          toggle_st ^= 1 ;
          digitalWrite(LED_BUILTIN, toggle_st);   // turn the LED on (HIGH is the voltage level)

        }
        break ;
      }

    case PROXIMITY_WAKEUP:
      {
        if ( (t_ProximityDebDur_ms >= PROXIMITY_SLEEP_TIME_MS) && (t_ProximityDebDur_ms < PROXIMITY_SHUTDOWN_TIME_MS) )
        {
          Serial.print("\n\t\t\t--- PROXIMITY Sleep---\n");

          digitalWrite(PROCESSOR_PIN, HIGH);   // send a pulse on Processor trigger Pin for 500 ms
          delay(500);
          digitalWrite(PROCESSOR_PIN, LOW);   // send a pulse on Processor trigger Pin for 500 ms

          digitalWrite(PROJECTOR_PIN, LOW);   // turn the Projector Pin on (HIGH is the voltage level)

          st_Proximity_SM_States.nextState = PROXIMTY_SLEEP ;              // Flag to indiate system initialized

          toggle_st ^= 1 ;
          digitalWrite(LED_BUILTIN, toggle_st);   // turn the LED on (HIGH is the voltage level)

        }
        else if (t_ProximityDebDur_ms >= PROXIMITY_SHUTDOWN_TIME_MS)
        {
          Serial.print("\n\t\t\t--- PROXIMITY Shutdown---\n");

          digitalWrite(PROCESSOR_PIN, HIGH);   // send a pulse on Processor trigger Pin for 500 ms
          delay(7000);
          digitalWrite(PROCESSOR_PIN, LOW);   // send a pulse on Processor trigger Pin for 500 ms

          digitalWrite(PROJECTOR_PIN, LOW);   // turn the Projector Pin on (HIGH is the voltage level)

          st_Proximity_SM_States.nextState = PROXIMITY_SHUTDOWN ;              // Flag to indiate system initialized

          toggle_st ^= 1 ;
          digitalWrite(LED_BUILTIN, toggle_st);   // turn the LED on (HIGH is the voltage level)

        }

        break ;
      }

    case PROXIMTY_SLEEP:
      {
        if (t_ProximityDebDur_ms >= PROXIMITY_WAKEUP_TIME_MS)
        {
          Serial.print("\n\t\t\t--- PROXIMITY Wakeup---\n");

          digitalWrite(PROCESSOR_PIN, HIGH);   // send a pulse on Processor trigger Pin for 500 ms
          delay(500);
          digitalWrite(PROCESSOR_PIN, LOW);   // send a pulse on Processor trigger Pin for 500 ms

          digitalWrite(PROJECTOR_PIN, HIGH);   // turn the Projector Pin on (HIGH is the voltage level)

          st_Proximity_SM_States.nextState = PROXIMITY_WAKEUP ;              // Flag to indiate system initialized

          toggle_st ^= 1 ;
          digitalWrite(LED_BUILTIN, toggle_st);   // turn the LED on (HIGH is the voltage level)

        }
        break ;
      }

    case PROXIMITY_SHUTDOWN:
      {
        Serial.print("\n\t\t\t--- PROXIMITY Shutdown---\n");

        digitalWrite(PROCESSOR_PIN, LOW);   // send a pulse on Processor trigger Pin for 500 ms
        digitalWrite(PROJECTOR_PIN, LOW);   // turn the Projector Pin on (HIGH is the voltage level)

        st_Proximity_SM_States.nextState = PROXIMITY_SHUTDOWN ;              // Flag to indiate system initialized

        toggle_st ^= 1 ;
        digitalWrite(LED_BUILTIN, toggle_st);   // turn the LED on (HIGH is the voltage level)

        break ;
      }

    default :
      break ;
  }

}

/**************************************************************************************************************************************/

void loop()
{
  static unsigned char u8_DebCntr_num       = 0  ;     // counter for Proximity debounce
  static unsigned char u8_ProcessorCntr_num = 0  ;     // counter for generating pulse to Processor trigger
  static bool b_TimeOut_st = FALSE ;
  static bool TO_toggle_LED_st = 0 ;

  //  static unsigned long t_DebounceStartTime_ms      = 0  ;     // the time the debounce for proximity was started
  //  static bool b_StartDeb_flag               = FALSE ;  // Proximity debounce start flag
  unsigned long int u32_ambient_lx          = 0  ;     // Ambient light in lux
  unsigned long int u32_distance_mm         = 0  ;     // object distance in mm

  uint8_t range = vl.readRange();
  uint8_t status = vl.readRangeStatus();
  
  if (status == VL6180X_ERROR_NONE) {
    Serial.print("Range: "); Serial.println(range);
  }
  
  // calculate the ambient linght in lux

  Serial.print("Ambient: ");
  u32_ambient_lx = sensor.readAmbientContinuous() ;
  Serial.print(u32_ambient_lx);
  if (sensor.timeoutOccurred())
  {
    Serial.print(" TIMEOUT");
    b_TimeOut_st = TRUE ;
  }
  else
    b_TimeOut_st = FALSE ;

  // calculate the distance in mm
  u32_distance_mm = sensor.readRangeContinuousMillimeters() ;
  Serial.print("\tDistance: ");
  Serial.print(u32_distance_mm );
  if (sensor.timeoutOccurred())
  {
    Serial.print(" TIMEOUT");
    b_TimeOut_st = TRUE ;
  }
  else
    b_TimeOut_st = FALSE ;

/*
  // if Some error occurred, print it out!
    Serial.println("\t\t");
  if  ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
    Serial.println("System error");
  }
  else if (status == VL6180X_ERROR_ECEFAIL) {
    Serial.println("ECE failure");
  }
  else if (status == VL6180X_ERROR_NOCONVERGE) {
    Serial.println("No convergence");
  }
  else if (status == VL6180X_ERROR_RANGEIGNORE) {
    Serial.println("Ignoring range");
  }
  else if (status == VL6180X_ERROR_SNR) {
    Serial.println("Signal/Noise error");
  }
  else if (status == VL6180X_ERROR_RAWUFLOW) {
    Serial.println("Raw reading underflow");
  }
  else if (status == VL6180X_ERROR_RAWOFLOW) {
    Serial.println("Raw reading overflow");
  }
  else if (status == VL6180X_ERROR_RANGEUFLOW) {
    Serial.println("Range reading underflow");
  }
  else if (status == VL6180X_ERROR_RANGEOFLOW) {
    Serial.println("Range reading overflow");
  }
  */
  
  /*/------------------------------------
    // evaluate proximity if object is close for more than defined rasters
    ( u32_distance_mm < PROXIMITY_THRESHOLD_MM) ? (u8_DebCntr_num++) : (u8_DebCntr_num = 0) ;

    if ( u8_DebCntr_num > PROXIMITY_DEB_CNT )
    {
    Set_IsNear_st(TRUE) ;
    u8_DebCntr_num = 0 ;
    }
    else
    {
    Set_IsNear_st(FALSE) ;
    }

    //------------------------------------

    // evaluate proximity if object is close for more than defined rasters
    if ( !b_StartDeb_flag && ( u32_distance_mm <= PROXIMITY_THRESHOLD_MM) )
    {
    t_DebounceStartTime_ms = millis()  ;
    b_StartDeb_flag = TRUE ;
    }
    else if ( u32_distance_mm <= PROXIMITY_THRESHOLD_MM)
    {
    // do debounce
    }
    else
    {
    b_StartDeb_flag = FALSE ;
    Set_IsNear_st(FALSE) ;
    }

    if ( (TRUE == b_StartDeb_flag) && (millis() - t_DebounceStartTime_ms > PROXIMITY_DEB_TIME_MS) )
    {
    Set_IsNear_st(TRUE) ;
    }

    //------------------------------------*/
    
//////////////////////////////////////////////////////////////////////////////////
//  if ( u32_distance_mm <= PROXIMITY_THRESHOLD_MM )
//    Set_IsNear_st(TRUE) ;
//  else
//    Set_IsNear_st(FALSE) ;
//////////////////////////////////////////////////////////////////////////////////

  /*****************************************************************/

  Serial.println();
  delay(100);

  while (b_TimeOut_st == TRUE )
  {
    TO_toggle_LED_st ^= 1 ;
    digitalWrite(LED_BUILTIN, TO_toggle_LED_st);   // turn the LED on (HIGH is the voltage level)
      delay(30);
  }
}

/**************************************************************************************************************************************/


