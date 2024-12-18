int low_voltage_flag = 1;
unsigned long vol_measure_time = 0;
void voltageInit()
{
  analogReference(INTERNAL1V1);
}

void voltageMeasure()
{
  if (millis() - vol_measure_time > 1000)
  {
    vol_measure_time = millis();
    double voltage = (analogRead(VOL_MEASURE_PIN) * 1.1 / 1024) * ((10 + 1.5) / 1.5);
    
    // Add these lines to print voltage
    //Serial.print("Voltage: ");
    //Serial.print(voltage);
    //Serial.println(" V");
  }
}