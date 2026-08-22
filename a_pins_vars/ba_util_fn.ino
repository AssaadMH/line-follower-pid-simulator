void printreadings(int delayBetweenEachPrint = 1000)
{
    delay(delayBetweenEachPrint);
    Serial.print("analog reading:  ");
    for (int i = 0; i < SensorCount; i++)
    {
        Serial.print(analogRead(analog_pins[i]));
        Serial.print(" ||| ");
    }
    Serial.print("    ");
    Serial.print(" L: ");
    Serial.print(ultrasonicL.read());
    Serial.print(" F: ");
    Serial.print(ultrasonicF.read());
   Serial.print(" R: ");
    Serial.print(ultrasonicR.read());
    Serial.println("      **");
}

boolean compare(uint16_t a[SensorCount], char b[SensorCount], int Length = 0)
{
    // FIX #6: compare exactly as many sensors as the pattern actually describes.
    // The pattern strings are plain C strings, so when no explicit Length is
    // given we use their real length (strlen) instead of the fixed SensorCount.
    // This stops 12-char patterns (e.g. "x1111111111x") from being compared
    // against 16 sensors, which previously made them never match.
    if (Length <= 0) Length = strlen(b);
    if (Length > SensorCount) Length = SensorCount;
    for (int i = 0; i < Length; i++)
    {
        if (b[i] == 'x')
        {
        }                                    // jump
        else if (a[i] != (((uint16_t)b[i]) - 48)) // -48 HOWA ASCII DE 0 POUR TRANFORMER CHARACTER TO INT just 0 and 1
            return false;
    }
    return true;
}

void myledwhiteon()
{
    ledactiontime = millis();
    for (int i = 0; i < NUM_LEDS; i++)
    {
        leds[i] = CHSV(0, 0, 255);
    }
    FastLED.show();
}
void delayGyro(int DELAY=50){
  double entry=millis();
  while(1){
    getAngle();
    delay(1);
    if(millis()-entry>=DELAY) break;
  }
}
