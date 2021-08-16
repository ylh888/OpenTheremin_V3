#include "Arduino.h"

#include "application.h"

#include "hw.h"
#include "SPImcpDAC.h"
#include "ihandlers.h"
#include "timer.h"
#include "EEPROM.h"

const AppMode AppModeValues[] = {MUTE, NORMAL};
const int16_t CalibrationTolerance = 15;
const int16_t PitchFreqOffset = 700;
const int16_t VolumeFreqOffset = 700;
const int8_t HYST_VAL = 40;

static int32_t pitchCalibrationBase = 0;
static int32_t pitchCalibrationBaseFreq = 0;
static int32_t pitchCalibrationConstant = 0;
static int32_t pitchSensitivityConstant = 70000;
static int16_t pitchDAC = 0;
static int16_t volumeDAC = 0;
static float qMeasurement = 0;

static int32_t volCalibrationBase   = 0;

Application::Application()
  : _state(PLAYING),
    _mode(NORMAL),
    _vt_cmd(NONE) {
};

void Application::setup() {
#if SERIAL_ENABLED
  Serial.begin(Application::BAUD);
#endif

  HW_LED1_ON; HW_LED2_OFF;

  pinMode(Application::BUTTON_PIN, INPUT_PULLUP);
  pinMode(Application::LED_PIN_1,    OUTPUT);
  pinMode(Application::LED_PIN_2,    OUTPUT);

  digitalWrite(Application::LED_PIN_1, HIGH);    // turn the LED off by making the voltage LOW

  SPImcpDACinit();

  EEPROM.get(0, pitchDAC);
  EEPROM.get(2, volumeDAC);

  SPImcpDAC2Asend(pitchDAC);
  SPImcpDAC2Bsend(volumeDAC);


  initialiseTimer();
  initialiseInterrupts();


  EEPROM.get(4, pitchCalibrationBase);
  EEPROM.get(8, volCalibrationBase);



}

void Application::initialiseTimer() {
  ihInitialiseTimer();
}

void Application::initialiseInterrupts() {
  ihInitialiseInterrupts();
}

void Application::InitialisePitchMeasurement() {
  ihInitialisePitchMeasurement();
}

void Application::InitialiseVolumeMeasurement() {
  ihInitialiseVolumeMeasurement();
}

unsigned long Application::GetQMeasurement()
{
  int qn = 0;

  TCCR1B = (1 << CS10);

  while (!(PIND & (1 << PORTD3)));
  while ((PIND & (1 << PORTD3)));

  TCNT1 = 0;
  timer_overflow_counter = 0;
  while (qn < 31250) {
    while (!(PIND & (1 << PORTD3)));
    qn++;
    while ((PIND & (1 << PORTD3)));
  };



  TCCR1B = 0;

  unsigned long frequency = TCNT1;
  unsigned long temp = 65536 * (unsigned long)timer_overflow_counter;
  frequency += temp;

  return frequency;

}


unsigned long Application::GetPitchMeasurement()
{
  TCNT1 = 0;
  timer_overflow_counter = 0;
  TCCR1B = (1 << CS12) | (1 << CS11) | (1 << CS10);

  delay(1000);

  TCCR1B = 0;

  unsigned long frequency = TCNT1;
  unsigned long temp = 65536 * (unsigned long)timer_overflow_counter;
  frequency += temp;

  return frequency;

}

unsigned long Application::GetVolumeMeasurement()
{ timer_overflow_counter = 0;

  TCNT0 = 0;
  TCNT1 = 49911;
  TCCR0B = (1 << CS02) | (1 << CS01) | (1 << CS00);	 // //External clock source on T0 pin. Clock on rising edge.
  TIFR1  = (1 << TOV1);      //Timer1 INT Flag Reg: Clear Timer Overflow Flag

  while (!(TIFR1 & ((1 << TOV1)))); // on Timer 1 overflow (1s)
  TCCR0B = 0;	 // Stop TimerCounter 0
  unsigned long frequency = TCNT0; // get counter 0 value
  unsigned long temp = (unsigned long)timer_overflow_counter; // and overflow counter

  frequency += temp * 256;

  return frequency;
}



#if CV_ENABLED                                 // Initialise PWM Generator for CV output
void initialiseCVOut() {

}
#endif

AppMode Application::nextMode() {
  return _mode == NORMAL ? MUTE : NORMAL; // AppModeValues[_mode + 1];
}

void Application::loop() {
  int32_t pitch_v = 0, pitch_l = 0;            // Last value of pitch  (for filtering)
  int32_t vol_v = 0,   vol_l = 0;              // Last value of volume (for filtering)

  uint16_t volumePotValue = 0;
  uint16_t pitchPotValue = 0;
  int registerPotValue, registerPotValueL = 0;
  int wavePotValue, wavePotValueL = 0;
  uint8_t registerValue = 2;
  uint16_t tmpVolume;

  unsigned long time_s = 0;

mloop:                   // Main loop avoiding the GCC "optimization"

  pitchPotValue    = analogRead(PITCH_POT);
  volumePotValue   = analogRead(VOLUME_POT);
  registerPotValue   = analogRead(REGISTER_SELECT_POT);
  wavePotValue = analogRead(WAVE_SELECT_POT);

  if ((registerPotValue - registerPotValueL) >= HYST_VAL || (registerPotValueL - registerPotValue) >= HYST_VAL) registerPotValueL = registerPotValue;
  if (((wavePotValue - wavePotValueL) >= HYST_VAL) || ((wavePotValueL - wavePotValue) >= HYST_VAL)) wavePotValueL = wavePotValue;

  vWavetableSelector = wavePotValueL >> 7;

  // New register pot configuration:
  // Left = -1 octave, Center = +/- 0, Right = +1 octave
  if (registerPotValue > 681)
  {
    registerValue = 1;
  } else if (registerPotValue < 342)
  {
    registerValue = 3;
  } else
  {
    registerValue = 2;
  }

  if (vt_ext) { // overwrite with external values
    vWavetableSelector = vt_vWavetableSelector;
    registerValue = vt_registerValue;
  }

  if (_state == PLAYING && HW_BUTTON_PRESSED) {
    _state = CALIBRATING;
    resetTimer();
    time_s = millis();
  }

  if (_state == CALIBRATING && HW_BUTTON_RELEASED) {

    // the sequence of the 'if/else' statements is important; longest checked first
    unsigned long milsecs = millis() - time_s;

    if (milsecs >= 4000) {

      HW_LED2_ON;

      playStartupSound();

      calibrate_pitch();
      calibrate_volume();


      initialiseTimer();
      initialiseInterrupts();

      playCalibratingCountdownSound();
      calibrate();

      _mode = NORMAL;
      HW_LED2_OFF;

      while (HW_BUTTON_PRESSED)
        ; // NOP

      _state = PLAYING;
      vt_ext = 0;

    } else if (milsecs >= 2000) { // long press of between 2sec and 4 sec switches to/from Visual Tuner/ Serial Control _state

      vt_ext = 1 - vt_ext;
      if (vt_ext) {
        vt_registerValue = registerValue;
        vt_vWavetableSelector = vWavetableSelector;
        vt_value = 0;
        _vt_cmd = NONE;
      } else {
      }
      
    } else { // one quick button press switches between Mute and Normal (sound)

      _mode = (_mode == MUTE) ? NORMAL : MUTE; // nextMode();
      if (_mode == NORMAL) {
        HW_LED1_ON;
        HW_LED2_OFF;
      } else {
        HW_LED1_OFF;
        HW_LED2_ON;
      };
      // playModeSettingSound();
    }
    _state = PLAYING;
  };



  if (pitchValueAvailable) {                        // If capture event

    pitch_v = pitch;                       // Averaging pitch values
    pitch_v = pitch_l + ((pitch_v - pitch_l) >> 2);
    pitch_l = pitch_v;


    //HW_LED2_ON;


    // set wave frequency for each mode
    switch (_mode) {
      case MUTE : /* NOTHING! */;                                        break;
      case NORMAL      : setWavetableSampleAdvance(((pitchCalibrationBase - pitch_v) + 2048 - (pitchPotValue << 2)) >> registerValue); break;
    };

    //  HW_LED2_OFF;

    pitchValueAvailable = false;
  }

  if (volumeValueAvailable) {
    vol = max(vol, 5000);

    vol_v = vol;                // Averaging volume values
    vol_v = vol_l + ((vol_v - vol_l) >> 2);
    vol_l = vol_v;

    switch (_mode) {
      case MUTE:  vol_v = 0;                                                      break;
      case NORMAL:      vol_v = MAX_VOLUME - (volCalibrationBase - vol_v) / 2 + (volumePotValue << 2) - 1024;                                     break;
    };

    // Limit and set volume value
    vol_v = min(vol_v, 4095);
    //    vol_v = vol_v - (1 + MAX_VOLUME - (volumePotValue << 2));
    vol_v = vol_v ;
    vol_v = max(vol_v, 0);
    tmpVolume = vol_v >> 4;

    // Give vScaledVolume a pseudo-exponential characteristic:
    vScaledVolume = tmpVolume * (tmpVolume + 2);

    volumeValueAvailable = false;
  }

#if CV_ENABLED
  OCR0A = pitch & 0xff;
#endif

#if SERIAL_ENABLED
  if (timerExpired(TICKS_100_MILLIS)) {
    resetTimer();

    if ( vt_ext || vt_show > 0) {
      uint16_t intended_freq = ((((pitchCalibrationBase - pitch_v) + 2048 - (pitchPotValue << 2)) * 1000) / 1024) >> registerValue;
      intended_freq = intended_freq > 5000 ? 0 : intended_freq;
      vt_loop(intended_freq);
    }
    /*
      Serial.write(pitch & 0xff);              // Send char on serial (if used)
      Serial.write((pitch >> 8) & 0xff);
    */

  }
#endif

  goto mloop;                           // End of main loop
}

void Application::calibrate()
{
  resetPitchFlag();
  resetTimer();
  savePitchCounter();
  while (!pitchValueAvailable && timerUnexpiredMillis(10))
    ; // NOP
  pitchCalibrationBase = pitch;
  pitchCalibrationBaseFreq = FREQ_FACTOR / pitchCalibrationBase;
  pitchCalibrationConstant = FREQ_FACTOR / pitchSensitivityConstant / 2 + 200;

  resetVolFlag();
  resetTimer();
  saveVolCounter();
  while (!volumeValueAvailable && timerUnexpiredMillis(10))
    ; // NOP
  volCalibrationBase = vol;


  EEPROM.put(4, pitchCalibrationBase);
  EEPROM.put(8, volCalibrationBase);

}

void Application::calibrate_pitch()
{

  static int16_t pitchXn0 = 0;
  static int16_t pitchXn1 = 0;
  static int16_t pitchXn2 = 0;
  static float q0 = 0;
  static long pitchfn0 = 0;
  static long pitchfn1 = 0;
  static long pitchfn = 0;

  Serial.begin(115200);
  Serial.println("\nPITCH CALIBRATION\n");

  HW_LED1_OFF;
  HW_LED2_ON;

  InitialisePitchMeasurement();
  interrupts();
  SPImcpDACinit();

  qMeasurement = GetQMeasurement();  // Measure Arudino clock frequency
  Serial.print("Arudino Freq: ");
  Serial.println(qMeasurement);

  q0 = (16000000 / qMeasurement * 500000); //Calculated set frequency based on Arudino clock frequency

  pitchXn0 = 0;
  pitchXn1 = 4095;

  pitchfn = q0 - PitchFreqOffset;      // Add offset calue to set frequency

  Serial.print("\nPitch Set Frequency: ");
  Serial.println(pitchfn);


  SPImcpDAC2Bsend(1600);

  SPImcpDAC2Asend(pitchXn0);
  delay(100);
  pitchfn0 = GetPitchMeasurement();

  SPImcpDAC2Asend(pitchXn1);
  delay(100);
  pitchfn1 = GetPitchMeasurement();

  Serial.print ("Frequency tuning range: ");
  Serial.print(pitchfn0);
  Serial.print(" to ");
  Serial.println(pitchfn1);


  while (abs(pitchfn0 - pitchfn1) > CalibrationTolerance) { // max allowed pitch frequency offset

    SPImcpDAC2Asend(pitchXn0);
    delay(100);
    pitchfn0 = GetPitchMeasurement() - pitchfn;

    SPImcpDAC2Asend(pitchXn1);
    delay(100);
    pitchfn1 = GetPitchMeasurement() - pitchfn;

    pitchXn2 = pitchXn1 - ((pitchXn1 - pitchXn0) * pitchfn1) / (pitchfn1 - pitchfn0); // new DAC value

    Serial.print("\nDAC value L: ");
    Serial.print(pitchXn0);
    Serial.print(" Freq L: ");
    Serial.println(pitchfn0);
    Serial.print("DAC value H: ");
    Serial.print(pitchXn1);
    Serial.print(" Freq H: ");
    Serial.println(pitchfn1);


    pitchXn0 = pitchXn1;
    pitchXn1 = pitchXn2;

    HW_LED2_TOGGLE;

  }
  delay(100);

  EEPROM.put(0, pitchXn0);

}

void Application::calibrate_volume()
{


  static int16_t volumeXn0 = 0;
  static int16_t volumeXn1 = 0;
  static int16_t volumeXn2 = 0;
  static float q0 = 0;
  static long volumefn0 = 0;
  static long volumefn1 = 0;
  static long volumefn = 0;

  Serial.begin(115200);
  Serial.println("\nVOLUME CALIBRATION");

  InitialiseVolumeMeasurement();
  interrupts();
  SPImcpDACinit();


  volumeXn0 = 0;
  volumeXn1 = 4095;

  q0 = (16000000 / qMeasurement * 460765);
  volumefn = q0 - VolumeFreqOffset;

  Serial.print("\nVolume Set Frequency: ");
  Serial.println(volumefn);


  SPImcpDAC2Bsend(volumeXn0);
  delay_NOP(44316);//44316=100ms

  volumefn0 = GetVolumeMeasurement();

  SPImcpDAC2Bsend(volumeXn1);

  delay_NOP(44316);//44316=100ms
  volumefn1 = GetVolumeMeasurement();


  Serial.print ("Frequency tuning range: ");
  Serial.print(volumefn0);
  Serial.print(" to ");
  Serial.println(volumefn1);


  while (abs(volumefn0 - volumefn1) > CalibrationTolerance) {

    SPImcpDAC2Bsend(volumeXn0);
    delay_NOP(44316);//44316=100ms
    volumefn0 = GetVolumeMeasurement() - volumefn;

    SPImcpDAC2Bsend(volumeXn1);
    delay_NOP(44316);//44316=100ms
    volumefn1 = GetVolumeMeasurement() - volumefn;

    volumeXn2 = volumeXn1 - ((volumeXn1 - volumeXn0) * volumefn1) / (volumefn1 - volumefn0); // calculate new DAC value

    Serial.print("\nDAC value L: ");
    Serial.print(volumeXn0);
    Serial.print(" Freq L: ");
    Serial.println(volumefn0);
    Serial.print("DAC value H: ");
    Serial.print(volumeXn1);
    Serial.print(" Freq H: ");
    Serial.println(volumefn1);


    volumeXn0 = volumeXn1;
    volumeXn1 = volumeXn2;
    HW_LED2_TOGGLE;

  }

  EEPROM.put(2, volumeXn0);

  HW_LED2_OFF;
  HW_LED1_ON;

  Serial.println("\nCALIBRATION COMPLETED\n");
}

void Application::hzToAddVal(float hz) {
  setWavetableSampleAdvance((uint16_t)(hz * HZ_ADDVAL_FACTOR));
}

void Application::playNote(float hz, uint16_t milliseconds = 500, uint8_t volume = 255) {
  vScaledVolume = volume * (volume + 2);
  hzToAddVal(hz);
  millitimer(milliseconds);
  vScaledVolume = 0;
}

void Application::playStartupSound() {
  playNote(MIDDLE_C, 150, 25);
  playNote(MIDDLE_C * 2, 150, 25);
  playNote(MIDDLE_C * 4, 150, 25);
}

void Application::playCalibratingCountdownSound() {
  playNote(MIDDLE_C * 2, 150, 25);
  playNote(MIDDLE_C * 2, 150, 25);
}

void Application::playModeSettingSound() {
  for (int i = 0; i <= _mode; i++) {
    playNote(MIDDLE_C * 2, 200, 25);
    millitimer(100);
  }
}

void Application::delay_NOP(unsigned long time) {
  volatile unsigned long i = 0;
  for (i = 0; i < time; i++) {
    __asm__ __volatile__ ("nop");
  }
}

/* ylh's visual tuner/controller

    interprets the following from the serial port
   "$Rn" = Register control where n = 1,2,3
   "$Wn" = Wavetable control where n = 0 ... 7
   "$sn" = shows info on serial whern n=0: none; n=1: pitch; n=2: debug details

*/
void Application::vt_loop(uint16_t f) {


  if ( vt_show == 1 ) {
    Serial.println(f);
  } else if (vt_show == 2) {
    Serial.print("\nplayed Hz:");
    Serial.print(f);
    vt_show_debug();
  } else {}

  while (Serial.available()) {
    int val = Serial.read();

    if (val == '$') {
      _vt_cmd = COMMAND;
      vt_value = 0;
    }

    else if ( _vt_cmd == COMMAND ) {
      if ( val == 'R' ) {
        _vt_cmd = REGISTER;
      } else if ( val == 'W' ) {
        _vt_cmd = WAVETABLE;
      } else if (val == 's' ) {
        _vt_cmd = SHOW;
      }
    }
    else if ( val >= '0' && val <= '9') {
      vt_value = vt_value * 10 + val - '0';
      if ( _vt_cmd == REGISTER ) {
        if (vt_value >= 1 && vt_value <= 3) {
          vt_registerValue = vt_value;
        } else {
          Serial.print( "illegal Register value " );
          Serial.print( vt_value );
        }
      }
      else if ( _vt_cmd == WAVETABLE ) {
        if (vt_value >= 0 && vt_value <= 7) {

          vt_vWavetableSelector = vt_value;
        } else {
          Serial.print( "illegal Wavetable value " );
          Serial.print( val );
        }
      }
      else if ( _vt_cmd == SHOW ) {
        if (vt_value >= 0 && vt_value <= 2) {
          vt_show = vt_value;
        } else {
          Serial.print( "illegal Show value " );
          Serial.print( vt_value );
        }
      }
      else {
        Serial.print("illegal input value: ");
        Serial.print(val);
      }
    }
  }
}

void Application::vt_show_debug() {

  Serial.print(" mode=");
  if ( _mode == NORMAL )
    Serial.print("Normal");
  else
    Serial.print("Mute");

  Serial.print(", state=");
  if ( _state == PLAYING )
    Serial.print("Playing");
  else
    Serial.print("Calibrating");

  Serial.print(" BUTTON="); Serial.print(HW_BUTTON_PRESSED);
  Serial.print(" timer="); Serial.print(timer);
  if ( vt_ext ) {
    Serial.print(" *EXT* " );
    Serial.print(" Reg= " ); Serial.print(vt_registerValue);
    Serial.print(" Wave= "); Serial.print(vt_vWavetableSelector);
  }

  Serial.print(", pitch=");
  Serial.print(pitch);
  Serial.print(", vol=");
  Serial.print(vol);

  Serial.print(" v=");  Serial.print(analogRead(VOLUME_POT));
  Serial.print(" p="); Serial.print(analogRead(PITCH_POT));

  Serial.print(" r="); Serial.print(analogRead(REGISTER_SELECT_POT));
  Serial.print(" w="); Serial.print(analogRead(WAVE_SELECT_POT));
  Serial.print(" ["); Serial.print(vWavetableSelector); Serial.print("]");

}
