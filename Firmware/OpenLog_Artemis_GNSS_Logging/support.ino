void printMajorDebug(String thingToPrint)
{
  if(settings.printMajorDebugMessages == true)
  {
    Serial.print(thingToPrint);    
  }
}

void printMinorDebug(String thingToPrint)
{
  if(settings.printMinorDebugMessages == true)
  {
    Serial.print(thingToPrint);    
  }
}

//Option not known
void printUnknown(uint8_t unknownChoice)
{
  Serial.print(F("Unknown choice: "));
  Serial.print(unknownChoice);
  Serial.println();
}
void printUnknown(int unknownValue)
{
  Serial.print(F("Unknown value: "));
  Serial.print(unknownValue);
  Serial.println();
}

//Blocking wait for user input
void waitForInput()
{
  for (int i = 0; i < 5; i++) //Wait for any incoming chars to hit buffer (let's keep this short!)
  {
    checkBattery();
    delay(1);
  }
  while (Serial.available() > 0) Serial.read(); //Clear buffer
  while (Serial.available() == 0)
  {
    storeData();
  }
}

//Get single byte from user
//Waits for and returns the character that the user provides
//Returns STATUS_GETNUMBER_TIMEOUT if input times out
//Returns 'x' if user presses 'x'
uint8_t getByteChoice(int numberOfSeconds, bool updateDZSERIAL)
{
  SerialFlush();

  for (int i = 0; i < 50; i++) //Wait for any incoming chars to hit buffer
  {
    checkBattery();
    delay(1);
  }
  
  while (Serial.available() > 0) Serial.read(); //Clear buffer

  if (settings.useTxRxPinsForTerminal == true)
    while (Serial1.available() > 0) Serial1.read(); //Clear buffer

  unsigned long startTime = millis();
  byte incoming;
  while (1)
  {
    if (Serial.available() > 0)
    {
      incoming = Serial.read();
      if (updateDZSERIAL)
      {
        DSERIAL = &Serial;
        ZSERIAL = &Serial;
      }
//      SerialPrint(F("byte: 0x"));
//      Serial.println(incoming, HEX);
//      if (settings.useTxRxPinsForTerminal == true)
//        Serial1.println(incoming, HEX);
      if (incoming >= 'a' && incoming <= 'z') break;
      if (incoming >= 'A' && incoming <= 'Z') break;
      if (incoming >= '0' && incoming <= '9') break;
    }

    if ((settings.useTxRxPinsForTerminal == true) && (Serial1.available() > 0))
    {
      incoming = Serial1.read();
      if (updateDZSERIAL)
      {
        DSERIAL = &Serial1;
        ZSERIAL = &Serial1;
      }
//      SerialPrint(F("byte: 0x"));
//      Serial.println(incoming, HEX);
//      if (settings.useTxRxPinsForTerminal == true)
//        Serial1.println(incoming, HEX);
      if (incoming >= 'a' && incoming <= 'z') break;
      if (incoming >= 'A' && incoming <= 'Z') break;
      if (incoming >= '0' && incoming <= '9') break;
    }

    if ( (millis() - startTime) / 1000 >= numberOfSeconds)
    {
      SerialPrintln(F("No user input received."));
      return (STATUS_GETBYTE_TIMEOUT); //Timeout. No user input.
    }

    checkBattery();
    delay(1);
  }

  return (incoming);
}

/*
//Get single byte from user
//Waits for and returns the character that the user provides
//Returns STATUS_GETNUMBER_TIMEOUT if input times out
//Returns 'x' if user presses 'x'
uint8_t getByteChoice(int numberOfSeconds)
{
  bool termOut = settings.enableTerminalOutput; //Store settings.enableTerminalOutput
  settings.enableTerminalOutput = false; //Disable terminal messages while waiting for a choice
  
  Serial.flush();
  for (int i = 0; i < 5; i++) //Wait for any incoming chars to hit buffer (let's keep this short!)
  {
    checkBattery();
    delay(1);
  }
  while (Serial.available() > 0) Serial.read(); //Clear buffer

  long startTime = millis();
  byte incoming;
  while (1)
  {
    if (Serial.available() > 0)
    {
      incoming = Serial.read();
//      Serial.print(F("byte: 0x"));
//      Serial.println(incoming, HEX);
      if (incoming >= 'a' && incoming <= 'z') break;
      if (incoming >= 'A' && incoming <= 'Z') break;
      if (incoming >= '0' && incoming <= '9') break;
    }

    if ( (millis() - startTime) / 1000 >= numberOfSeconds)
    {
      Serial.println(F("No user input received."));
      settings.enableTerminalOutput = termOut; //Restore settings.enableTerminalOutput
      return (STATUS_GETBYTE_TIMEOUT); //Timeout. No user input.
    }

    storeData(); //Keep reading I2C data and writing it to SD

    checkBattery();
    delay(1);
  }

  settings.enableTerminalOutput = termOut; //Restore settings.enableTerminalOutput
  return (incoming);
}
*/

//Get a string/value from user, remove all non-numeric values
//Returns STATUS_GETNUMBER_TIMEOUT if input times out
//Returns STATUS_PRESSED_X if user presses 'x'
int64_t getNumber(int numberOfSeconds)
{
  bool termOut = settings.enableTerminalOutput; //Store settings.enableTerminalOutput
  settings.enableTerminalOutput = false; //Disable terminal messages while waiting for a number
  
  for (int i = 0; i < 5; i++) //Wait for any incoming chars to hit buffer (let's keep this short!)
  {
    checkBattery();
    delay(1);
  }
  while (Serial.available() > 0) Serial.read(); //Clear buffer

  //Get input from user
  char cleansed[20]; //Good for very large numbers: 123,456,789,012,345,678\0

  long startTime = millis();
  int spot = 0;
  while (spot < 20 - 1) //Leave room for terminating \0
  {
    while (Serial.available() == 0) //Wait for user input
    {
      checkBattery();
      
      if ( (millis() - startTime) / 1000 >= numberOfSeconds)
      {
        if (spot == 0)
        {
          Serial.println(F("No user input received. Do you have line endings turned on?"));
          settings.enableTerminalOutput = termOut; //Restore settings.enableTerminalOutput
          return (STATUS_GETNUMBER_TIMEOUT); //Timeout. No user input.
        }
        else if (spot > 0)
        {
          break; //Timeout, but we have data
        }
      }

      storeData(); //Keep reading I2C data and writing it to SD
    }

    //See if we timed out waiting for a line ending
    if (spot > 0 && (millis() - startTime) / 1000 >= numberOfSeconds)
    {
      Serial.println(F("Do you have line endings turned on?"));
      break; //Timeout, but we have data
    }

    byte incoming = Serial.read();
    if (incoming == '\n' || incoming == '\r')
    {
      Serial.println();
      break;
    }

    if ((isDigit(incoming) == true) || ((incoming == '-') && (spot == 0))) // Check for digits and a minus sign
    {
      Serial.write(incoming); //Echo user's typing
      cleansed[spot++] = (char)incoming;
    }

    if (incoming == 'x')
    {
      settings.enableTerminalOutput = termOut; //Restore settings.enableTerminalOutput
      return (STATUS_PRESSED_X);
    }
  }

  cleansed[spot] = '\0';

  int64_t largeNumber = 0;
  int x = 0;
  if (cleansed[0] == '-') // If our number is negative
  {
    x = 1; // Skip the minus
  }
  for( ; x < spot ; x++)
  {
    largeNumber *= 10;
    largeNumber += (cleansed[x] - '0');
  }
  if (cleansed[0] == '-') // If our number is negative
  {
    largeNumber = 0 - largeNumber; // Make it negative
  }

  settings.enableTerminalOutput = termOut; //Restore settings.enableTerminalOutput
  return (largeNumber);
}

//Get a string/value from user, remove all non-numeric values
//Returns STATUS_GETNUMBER_TIMEOUT if input times out
//Returns STATUS_PRESSED_X if user presses 'x'
double getDouble(int numberOfSeconds)
{
  for (int i = 0; i < 10; i++) //Wait for any incoming chars to hit buffer
  {
    checkBattery();
    delay(1);
  }
  while (Serial.available() > 0) Serial.read(); //Clear buffer

  //Get input from user
  char cleansed[20]; //Good for very large numbers: 123,456,789,012,345,678\0

  long startTime = millis();
  int spot = 0;
  bool dpSeen = false;
  while (spot < 20 - 1) //Leave room for terminating \0
  {
    while (Serial.available() == 0) //Wait for user input
    {
      checkBattery();
      
      if ( (millis() - startTime) / 1000 >= numberOfSeconds)
      {
        if (spot == 0)
        {
          Serial.println(F("No user input received. Do you have line endings turned on?"));
          return (STATUS_GETNUMBER_TIMEOUT); //Timeout. No user input.
        }
        else if (spot > 0)
        {
          break; //Timeout, but we have data
        }
      }
    }

    //See if we timed out waiting for a line ending
    if (spot > 0 && (millis() - startTime) / 1000 >= numberOfSeconds)
    {
      Serial.println(F("Do you have line endings turned on?"));
      break; //Timeout, but we have data
    }

    byte incoming = Serial.read();
    if (incoming == '\n' || incoming == '\r')
    {
      Serial.println();
      break;
    }

    if ((isDigit(incoming) == true) || ((incoming == '-') && (spot == 0)) || ((incoming == '.') && (dpSeen == false))) // Check for digits/minus/dp
    {
      Serial.write(incoming); //Echo user's typing
      cleansed[spot++] = (char)incoming;
    }

    if (incoming == '.')
      dpSeen = true;

    if (incoming == 'x')
    {
      return (STATUS_PRESSED_X);
    }
  }

  cleansed[spot] = '\0';

  double largeNumber = 0;
  int x = 0;
  if (cleansed[0] == '-') // If our number is negative
  {
    x = 1; // Skip the minus
  }
  for( ; x < spot ; x++)
  {
    if (cleansed[x] == '.')
      break;
    largeNumber *= 10;
    largeNumber += (cleansed[x] - '0');
  }
  if (x < spot) // Check if we found a '.'
  {
    x++;
    double divider = 0.1;
    for( ; x < spot ; x++)
    {
      largeNumber += (cleansed[x] - '0') * divider;
      divider /= 10;
    }
  }
  if (cleansed[0] == '-') // If our number is negative
  {
    largeNumber = 0 - largeNumber; // Make it negative
  }
  return (largeNumber);
}

//*****************************************************************************
//
//  Divide an unsigned 32-bit value by 10.
//
//  Note: Adapted from Ch10 of Hackers Delight (hackersdelight.org).
//
//*****************************************************************************
static uint64_t divu64_10(uint64_t ui64Val)
{
    uint64_t q64, r64;
    uint32_t q32, r32, ui32Val;

    //
    // If a 32-bit value, use the more optimal 32-bit routine.
    //
    if ( ui64Val >> 32 )
    {
        q64 = (ui64Val>>1) + (ui64Val>>2);
        q64 += (q64 >> 4);
        q64 += (q64 >> 8);
        q64 += (q64 >> 16);
        q64 += (q64 >> 32);
        q64 >>= 3;
        r64 = ui64Val - q64*10;
        return q64 + ((r64 + 6) >> 4);
    }
    else
    {
        ui32Val = (uint32_t)(ui64Val & 0xffffffff);
        q32 = (ui32Val>>1) + (ui32Val>>2);
        q32 += (q32 >> 4);
        q32 += (q32 >> 8);
        q32 += (q32 >> 16);
        q32 >>= 3;
        r32 = ui32Val - q32*10;
        return (uint64_t)(q32 + ((r32 + 6) >> 4));
    }
}

//*****************************************************************************
//
// Converts ui64Val to a string.
// Note: pcBuf[] must be sized for a minimum of 21 characters.
//
// Returns the number of decimal digits in the string.
//
// NOTE: If pcBuf is NULL, will compute a return ui64Val only (no chars
// written).
//
//*****************************************************************************
static int uint64_to_str(uint64_t ui64Val, char *pcBuf)
{
    char tbuf[25];
    int ix = 0, iNumDig = 0;
    unsigned uMod;
    uint64_t u64Tmp;

    do
    {
        //
        // Divide by 10
        //
        u64Tmp = divu64_10(ui64Val);

        //
        // Get modulus
        //
        uMod = ui64Val - (u64Tmp * 10);

        tbuf[ix++] = uMod + '0';
        ui64Val = u64Tmp;
    } while ( ui64Val );

    //
    // Save the total number of digits
    //
    iNumDig = ix;

    //
    // Now, reverse the buffer when saving to the caller's buffer.
    //
    if ( pcBuf )
    {
        while ( ix-- )
        {
            *pcBuf++ = tbuf[ix];
        }

        //
        // Terminate the caller's buffer
        //
        *pcBuf = 0x00;
    }

    return iNumDig;
}

//*****************************************************************************
//
//  Float to ASCII text. A basic implementation for providing support for
//  single-precision %f.
//
//  param
//      fValue     = Float value to be converted.
//      pcBuf      = Buffer to place string AND input of buffer size.
//      iPrecision = Desired number of decimal places.
//      bufSize    = The size (in bytes) of the buffer.
//                   The recommended size is at least 16 bytes.
//
//  This function performs a basic translation of a floating point single
//  precision value to a string.
//
//  return Number of chars printed to the buffer.
//
//*****************************************************************************
#define OLA_FTOA_ERR_VAL_TOO_SMALL   -1
#define OLA_FTOA_ERR_VAL_TOO_LARGE   -2
#define OLA_FTOA_ERR_BUFSIZE         -3

typedef union
{
    int32_t I32;
    float F;
} ola_i32fl_t;

static int olaftoa(float fValue, char *pcBuf, int iPrecision, int bufSize)
{
    ola_i32fl_t unFloatValue;
    int iExp2, iBufSize;
    int32_t i32Significand, i32IntPart, i32FracPart;
    char *pcBufInitial, *pcBuftmp;

    iBufSize = bufSize; // *(uint32_t*)pcBuf;
    if (iBufSize < 4)
    {
        return OLA_FTOA_ERR_BUFSIZE;
    }

    if (fValue == 0.0f)
    {
        // "0.0"
        *(uint32_t*)pcBuf = 0x00 << 24 | ('0' << 16) | ('.' << 8) | ('0' << 0);
        return 3;
    }

    pcBufInitial = pcBuf;

    unFloatValue.F = fValue;

    iExp2 = ((unFloatValue.I32 >> 23) & 0x000000FF) - 127;
    i32Significand = (unFloatValue.I32 & 0x00FFFFFF) | 0x00800000;
    i32FracPart = 0;
    i32IntPart = 0;

    if (iExp2 >= 31)
    {
        return OLA_FTOA_ERR_VAL_TOO_LARGE;
    }
    else if (iExp2 < -23)
    {
        return OLA_FTOA_ERR_VAL_TOO_SMALL;
    }
    else if (iExp2 >= 23)
    {
        i32IntPart = i32Significand << (iExp2 - 23);
    }
    else if (iExp2 >= 0)
    {
        i32IntPart = i32Significand >> (23 - iExp2);
        i32FracPart = (i32Significand << (iExp2 + 1)) & 0x00FFFFFF;
    }
    else // if (iExp2 < 0)
    {
        i32FracPart = (i32Significand & 0x00FFFFFF) >> -(iExp2 + 1);
    }

    if (unFloatValue.I32 < 0)
    {
        *pcBuf++ = '-';
    }

    if (i32IntPart == 0)
    {
        *pcBuf++ = '0';
    }
    else
    {
        if (i32IntPart > 0)
        {
            uint64_to_str(i32IntPart, pcBuf);
        }
        else
        {
            *pcBuf++ = '-';
            uint64_to_str(-i32IntPart, pcBuf);
        }
        while (*pcBuf)    // Get to end of new string
        {
            pcBuf++;
        }
    }

    //
    // Now, begin the fractional part
    //
    *pcBuf++ = '.';

    if (i32FracPart == 0)
    {
        *pcBuf++ = '0';
    }
    else
    {
        int jx, iMax;

        iMax = iBufSize - (pcBuf - pcBufInitial) - 1;
        iMax = (iMax > iPrecision) ? iPrecision : iMax;

        for (jx = 0; jx < iMax; jx++)
        {
            i32FracPart *= 10;
            *pcBuf++ = (i32FracPart >> 24) + '0';
            i32FracPart &= 0x00FFFFFF;
        }

        //
        // Per the printf spec, the number of digits printed to the right of the
        // decimal point (i.e. iPrecision) should be rounded.
        // Some examples:
        // Value        iPrecision          Formatted value
        // 1.36399      Unspecified (6)     1.363990
        // 1.36399      3                   1.364
        // 1.36399      4                   1.3640
        // 1.36399      5                   1.36399
        // 1.363994     Unspecified (6)     1.363994
        // 1.363994     3                   1.364
        // 1.363994     4                   1.3640
        // 1.363994     5                   1.36399
        // 1.363995     Unspecified (6)     1.363995
        // 1.363995     3                   1.364
        // 1.363995     4                   1.3640
        // 1.363995     5                   1.36400
        // 1.996        Unspecified (6)     1.996000
        // 1.996        2                   2.00
        // 1.996        3                   1.996
        // 1.996        4                   1.9960
        //
        // To determine whether to round up, we'll look at what the next
        // decimal value would have been.
        //
        if ( ((i32FracPart * 10) >> 24) >= 5 )
        {
            //
            // Yes, we need to round up.
            // Go back through the string and make adjustments as necessary.
            //
            pcBuftmp = pcBuf - 1;
            while ( pcBuftmp >= pcBufInitial )
            {
                if ( *pcBuftmp == '.' )
                {
                }
                else if ( *pcBuftmp == '9' )
                {
                    *pcBuftmp = '0';
                }
                else
                {
                    *pcBuftmp += 1;
                    break;
                }
                pcBuftmp--;
            }
        }
    }

    //
    // Terminate the string and we're done
    //
    *pcBuf = 0x00;

    return (pcBuf - pcBufInitial);
} // olaftoa()

//Query the RTC and put the appropriately formatted (according to settings) 
//string into the passed buffer. timeStringBuffer should be at least 37 chars long
//Code modified by @DennisMelamed in PR #70
void getTimeString(char timeStringBuffer[])
{
  //reset the buffer
  timeStringBuffer[0] = '\0';

  myRTC.getTime();

  char rtcDate[12];
  char rtcDay[3];
  char rtcMonth[3];
  char rtcYear[5];
  if (myRTC.dayOfMonth < 10)
    sprintf(rtcDay, "0%d", myRTC.dayOfMonth);
  else
    sprintf(rtcDay, "%d", myRTC.dayOfMonth);
  if (myRTC.month < 10)
    sprintf(rtcMonth, "0%d", myRTC.month);
  else
    sprintf(rtcMonth, "%d", myRTC.month);
  if (myRTC.year < 10)
    sprintf(rtcYear, "200%d", myRTC.year);
  else
    sprintf(rtcYear, "20%d", myRTC.year);
  sprintf(rtcDate, "%s/%s/%s", rtcYear, rtcMonth, rtcDay);
  strcat(timeStringBuffer, rtcDate);

  char rtcTime[13];
  int adjustedHour = myRTC.hour;
  char rtcHour[3];
  char rtcMin[3];
  char rtcSec[3];
  char rtcHundredths[3];
  if (adjustedHour < 10)
    sprintf(rtcHour, "0%d", adjustedHour);
  else
    sprintf(rtcHour, "%d", adjustedHour);
  if (myRTC.minute < 10)
    sprintf(rtcMin, "0%d", myRTC.minute);
  else
    sprintf(rtcMin, "%d", myRTC.minute);
  if (myRTC.seconds < 10)
    sprintf(rtcSec, "0%d", myRTC.seconds);
  else
    sprintf(rtcSec, "%d", myRTC.seconds);
  if (myRTC.hundredths < 10)
    sprintf(rtcHundredths, "0%d", myRTC.hundredths);
  else
    sprintf(rtcHundredths, "%d", myRTC.hundredths);
  sprintf(rtcTime, " %s:%s:%s.%s", rtcHour, rtcMin, rtcSec, rtcHundredths);
  strcat(timeStringBuffer, rtcTime);
}
