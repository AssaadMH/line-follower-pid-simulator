// ============================================================================
//  TEMPORARY CALIBRATION HELPER  --  measures robot_sim.py CFG constants
//
//  *** THIS WHOLE FILE IS TEMPORARY. DELETE IT WHEN CALIBRATION IS DONE. ***
//  Also remove the `calibrate_all();` line in y_setup.ino (added between
//  calibratesensors(); and waitForStartButton();).
//
//  Named zz_... so it sorts last as a tab. It is auto-compiled with the sketch,
//  so you do NOT need to paste anything. Upload, open Serial @ 115200, follow
//  the prompts. Uses your existing forward()/stope()/updatesensors()/pidfollow()
//  and globals position / direction / offD / currentLineColor / BLOCKMOVEMENT.
// ============================================================================

void calib_loopTime() {
  Serial.println(F("\n[2] LOOP TIME (wheels blocked, no motion)..."));
  BLOCKMOVEMENT = true;
  const int N = 300;
  unsigned long t0 = millis();
  for (int i = 0; i < N; i++) {
    updatesensors('B', direction, offD);
    pidfollow(0);
  }
  unsigned long dt = millis() - t0;
  BLOCKMOVEMENT = false;
  stope();
  Serial.print(F("    --> loop_dt_ms = "));
  Serial.println((float)dt / N, 2);
}

void calib_setPoint() {
  Serial.println(F("\n[3] SETPOINT: center the robot EXACTLY on the line by hand."));
  Serial.println(F("    Watch 'position'. Its steady value = SetPoint. (runs 10s)"));
  unsigned long t0 = millis();
  while (millis() - t0 < 10000) {
    updatesensors(currentLineColor, direction, offD);
    Serial.print(F("    position = "));
    Serial.println(position);
    delay(300);
  }
}

void calib_topSpeed() {
  Serial.println(F("\n[1] TOP SPEED: robot drives STRAIGHT for 1.0s at PWM 255."));
  Serial.println(F("    Put it on the floor with clear space ahead. Go in 4s..."));
  delay(4000);
  forward(255, 255);
  delay(1000);
  stope();
  Serial.println(F("    DONE. Measure distance travelled (cm) = v_at_pwm255_cmps"));
}

void calibrate_all() {
  Serial.println(F("\n======== CALIBRATION HELPER ========"));
  calib_loopTime();   // safe: no motion
  calib_setPoint();   // safe: you position the robot by hand
  calib_topSpeed();   // !!! MOTION !!! clear space ahead
  Serial.println(F("\n======== DONE -> put values into robot_sim.py CFG ========"));
  while (1) { /* halt here so nothing else runs */ }
}
