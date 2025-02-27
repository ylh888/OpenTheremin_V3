#ifndef _APPLICATION_H
#define _APPLICATION_H

#include <avr/io.h>

#include "build.h"

enum AppState {CALIBRATING = 0, PLAYING};
enum AppMode  {MUTE = 0, NORMAL};

enum AppCmd { NONE, COMMAND, REGISTER, WAVETABLE, SHOW };

class Application {
  public:
    Application();

    void setup();
    void loop();

  private:
    static const uint16_t MAX_VOLUME = 4095;
    static const uint32_t TRIM_PITCH_FACTOR = 33554432;
    static const uint32_t FREQ_FACTOR = 1600000000;

    static const int16_t BUTTON_PIN = 6;
    static const int16_t LED_PIN_1  = 18;
    static const int16_t LED_PIN_2  = 19;


    static const int16_t PITCH_POT = 0;
    static const int16_t VOLUME_POT = 1;
    static const int16_t WAVE_SELECT_POT = 2;
    static const int16_t REGISTER_SELECT_POT = 3;

    int vt_ext = 0; // whether to use external control
    int vt_show = 1; // send to serial: 0 - none; 1 - frequency; 2 - details
    uint16_t vt_value = 0; // var to construct incoming values
    uint8_t vt_registerValue = 2;
    uint8_t vt_vWavetableSelector = 0;

#if SERIAL_ENABLED
    static const int BAUD = 115200;
#endif

    AppState _state;
    AppMode  _mode;
    AppCmd   _vt_cmd;

    void calibrate();
    void calibrate_pitch();
    void calibrate_volume();


    AppMode nextMode();

    void initialiseTimer();
    void initialiseInterrupts();
    void InitialisePitchMeasurement();
    void InitialiseVolumeMeasurement();
    unsigned long GetPitchMeasurement();
    unsigned long GetVolumeMeasurement();
    unsigned long GetQMeasurement();


    const float HZ_ADDVAL_FACTOR = 2.09785;
    const float MIDDLE_C = 261.6;

    void playNote(float hz, uint16_t milliseconds, uint8_t volume);
    void hzToAddVal(float hz);
    void playStartupSound();
    void playCalibratingCountdownSound();
    void playModeSettingSound();
    void delay_NOP(unsigned long time);

    void vt_loop(uint16_t);
    void vt_show_debug();
};

#endif // _APPLICATION_H
