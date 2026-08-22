void setup()
{
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  FastLED.addLeds<WS2811, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(80);
  stope();
  qtr.emittersOff();
  qtr.setTypeAnalog(); // or setTypeAnalog()
  qtr.setSensorPins(analog_pins, SensorCount);
  lcd.begin(16, 2);
  lcd.clear();
  //// *********************************************************
   Serial.begin(115200); //  //// DONT FORGOT TO DISABLE THIS BEFORE STARTING THE ROBOT *********************************************************
  Wire.begin();
  mpu.begin();
  mpu.calcGyroOffsets();
 //forward(110,110);
 //delay(300);
 //stope();
//delay(300);
//forward(-110,110);
//delay(300);
  //turn(90);
  initialCheck();
  initialCalcTimes();
  calibratesensors();
  calibrate_all();   // <-- TEMP CALIBRATION HELPER: runs the 3 on-robot tests, then halts. REMOVE THIS LINE + the pasted helper block when done.
  waitForStartButton();
  //BLOCKMOVEMENT = true;
  stope();
  getAngle();
  initialAngle = GyroAngle;
  otherconditionsCounter=0;
// forward(130, 130);
// delay(100);
 // otherconditionsCounter=5;
}
