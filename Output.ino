
void PrintData(void)
{    
      #if PRINT_JSON == 1
      Serial.print("{\"attitude\":{\"roll\":");
      Serial.print(RADTODEG * roll);
      Serial.print(",\"pitch\":");
      Serial.print(RADTODEG * pitch);
      Serial.print(",\"heading\":");
      Serial.print(RADTODEG * headingAngle);
      Serial.print("},\"altitude\":");
      Serial.print(baroAlt);
      Serial.print(",\"g\":");
      Serial.print(gLoad);
      Serial.print(",\"slip\":");
      Serial.print(RADTODEG * slipAngle);
      Serial.print(",\"turnRate\":");
      Serial.print(RADTODEG * turnRate);
      Serial.print("}");
      #endif
      
      #if PRINT_LEVIL == 1
      Serial.print("$RPYL,");
      Serial.print(10 * RADTODEG * roll, 0);
      Serial.print(",");
      Serial.print(10 * RADTODEG * pitch, 0);
      Serial.print(",");
      Serial.print(10 * RADTODEG * (headingAngle < 0 ? headingAngle + 6.28 : headingAngle), 0);
      Serial.print(",");
      Serial.print(10 * RADTODEG * slipAngle, 0);
      Serial.print(",");
      Serial.print(10 * RADTODEG * turnRate, 0);
      Serial.print(",");
      Serial.print(1000  * gLoad, 0);
      Serial.print(",");
      Serial.print("0");
      Serial.print(",");
      #endif
      
      #if CALIBRATE_MAG == 1
      Serial.print(magRaw.x);
      Serial.print(",");
      Serial.print(magRaw.y);
      Serial.print(",");  
      Serial.print(magRaw.z);
      Serial.print(",");
      R.print();
      Serial.print(",");
      Serial.print(angleTurned);
      #endif
      Serial.println();
}

void PrintBaro(void) {
      # if PRINT_LEVILBARO == 1
      Serial.print("$APENV1,");
      Serial.print( sqrt(rollRate*rollRate +pitchRate*pitchRate)* 60 * 57, 0);
      Serial.print(",");
      Serial.print(baroAlt);
      Serial.print(",0,0,0,");
      Serial.print(verticalSpeed);
      Serial.print(",");
      Serial.print(0);
      Serial.println(",");
      #endif
}

void PrintPower(void) {
      Serial.print("$APPOWER,");
      Serial.print(voltage,1);
      Serial.print(",0,");
      Serial.print(battery);
      Serial.println(",");
}
/*sqrt(pitchRate * pitchRate + rollRate * rollRate)*/
