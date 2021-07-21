## Open.Theremin V3.1 with Visual Tuner

This version allows the Open Theremin to interact with a Visual Tuner, [a web based software](https://ylh888.github.io/art/visualtuner.html).

The pitch information is displayed in the browser, which can also be used to select the wave table and register, bypassing the Theremin's knobs.

1. Connect the Theremin by the USB cable to the computer.

2. You need to use one of the following browsers: Chrome, Chromium, Edge or Opera. In addition, on these browsers you must enable experimental features. To do this copy one of:

- chrome://flags/#enable-experimental-web-platform-features
- edge://flags/#enable-experimental-web-platform-features
- opera://flags/#enable-experimental-web-platform-features

and paste it into your brower's address bar. Then, set the option to 'Enabled'.

3. Now, in a browser window, run [the Visual Tuner](https://ylh888.github.io/art/visualtuner.html).

Click anywhere on the canvas and select the port which connects to the Theremin. 

The Theremin will begin to display the pitch information. There are three different presentation styles.

4. On the Theremin, the button works as follows.

- press and release quickly switches between Mute and Normal;
- long press for 2 to 4 secs (then release)  will switch between local control (using the knobs) or through the browser;
- long press for more than 4 secs (then release) will put the theremin into Calibration state. Following Calibration, the Theremin returns to Play state.

This software is derived from:-

## Open.Theremin V3.1 control software

Arduino UNO Software for the Open.Theremin

### Changes since V3.0 (all by @Theremingenieur):
1. Fix a wavetable addressing issue (found by @miguelfreitas)
2. Use the Arduino's hardware SPI to control the DACS and use the Latch signal to reduce audio jitter
3. Improve the register switch to transpose by clean octaves and keep the tone spacing and pitch tuning consistent
4. Improve the volume response to give a smoother start and wider dynamics (*)

(*) This relies on a recent gcc compiler version. Make sure to compile it with the Arduino IDE >= 1.8.10

### Don't click on the files!
Click on the "Download ZIP" Button to the right or [Click here](https://github.com/GaudiLabs/OpenTheremin_V3/archive/master.zip) 
Then unpack the archive.

### Open Source Theremin based on the Arduino Platform

Open.Theremin is an arduino shield to build the legendary music instrument invented by Leon Theremin back in 1920. The theremin is played with two antennas, one to control the pitch and one for volume. The electronic shield with two ports to connect those antennas comprises two heterodyne oscillators to measure the distance of the hand to the antenna when playing the instrument. The resulting signal is fed into the arduino. After linearization and filtering the arduino generates the instruments sound that is then played through a high quality digital analog audio converter on the board. The characteristics of the sound can be determined by a wave table on the arduino.

For more info on the open source project and on availability of ready made shield see:

http://www.gaudi.ch/OpenTheremin/

### Installation
1. Open up the Arduino IDE
2. Open the File "Open_Theremin_V3.ino"
3. Selecting the correct usb port on Tools -> Serial Port
4. Select the correct arduino board from Tools -> Board
5. Upload the code by clicking on the upload button.

### LICENSE
Written by Urs Gaudenz, GaudiLabs, 2016
GNU license, check LICENSE file for more information
All text above must be included in any redistribution

