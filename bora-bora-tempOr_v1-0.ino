#include <LSM9DS1_Registers.h>
#include <LSM9DS1_Types.h>
#include <SparkFunLSM9DS1.h>

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <RTClib.h>


RTC_DS3231 rtc;
LSM9DS1 imu;
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

boolean SerialStat = false;
boolean SDStat = false;
boolean RTC_Valid = 0;

//Battery measurement.
#define vBatPin A7 // A7 for the M0 board; A9 for 32U4.

//temperature measurement.
#define temperature1Pin A2 // AD590
#define temperature2Pin A3 // LM19 temp. sensor
#define temperature3Pin A4 // LM19 temp. sensor

// Set the pins used for the SD card **
#define cardSelect 4  
char filename[15];
File logfile;
char CardExists;

// Built-in LEDs **********************
#define redLED 13       //Double checked for m0.
#define greenLED 8      //Double checked for m0. 
int blueLED = 12; 

//chip select for the rtc *************
const int  RTC_cs=11;      //19; 
String lFile;

//Address of current monitor **********
const int addr = 0x40;

// values from the current sensors ****
static int iShunt;
static int iVoltage;


/*
 * Some file dump and directory routines.
 */
File root;
File entry;
File dataFile;

//*******************************************
void DumpFile (void)
{
    Serial.print ("Dump file ");
    Serial.println (lFile);

  
 // if (!SD.begin (cardSelect)) {Serial.println ("No SD"); return;}
  dataFile = SD.open(lFile);

  
  // if the file is available, write to it:
  if (dataFile) 
  {
    while (dataFile.available()) {
      Serial.write(dataFile.read());
    }
    dataFile.close();
  }
  else Serial.println ("failed to open");

  Serial.println("============================================");
}

//*******************************************
void printDirectory(File dir, int numTabs) {
  Serial.println ("============================================");
  Serial.println ("Current Directory Listing");
  dir.rewindDirectory();
  while (true) 
  {
    entry =  dir.openNextFile();
    if (! entry) 
    {
      // no more files
      break;
    }

    for (uint8_t i = 0; i < numTabs; i++) 
    {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } 
    else 
    {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
      lFile = entry.name();
    entry.close();

    
  }
}

 //*************************************************************************************
//*******************************************
void ListDirectory (void)
{
/*
  Serial.print("Initializing SD card...");

  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
*/
  //SD.begin(4);
  root = SD.open("/");

  printDirectory(root, 0);
  root.close();

  Serial.println("============================================");
}




 //*************************************************************************************
 //*************************************************************************************
int SetTimeDate(int d, int mo, int y, int h, int mi, int s){ 
  int TimeDate [7]={s,mi,h,0,d,mo,y};
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE1);
  for(int i=0; i<=6;i++){
    if(i==3)
      i++;
    int b= TimeDate[i]/10;
    int a= TimeDate[i]-b*10;
    if(i==2){
      if (b==2)
        b=B00000010;
      else if (b==1)
        b=B00000001;
    } 
    TimeDate[i]= a+(b<<4);
      
    digitalWrite(RTC_cs, LOW);
    SPI.transfer(i+0x80); 
    SPI.transfer(TimeDate[i]);        
    digitalWrite(RTC_cs, HIGH);
  }
}

 //*************************************************************************************
 //*************************************************************************************
int RTC_init(){ 
    Serial.println ("RTC_init started");
    pinMode(RTC_cs,OUTPUT); // chip select
    // start the SPI library:
    digitalWrite (RTC_cs, LOW);
    //SPI.begin(RTC_cs);
    SPI.setBitOrder(MSBFIRST); 
    SPI.setDataMode(SPI_MODE1); // both mode 1 & 3 should work 
    //set control register 
    digitalWrite(RTC_cs, LOW);  
    SPI.transfer(0x8E);
    SPI.transfer(0x60); //60= disable Oscillator and Battery SQ wave @1hz, temp compensation, Alarms disabled
    digitalWrite(RTC_cs, HIGH);
    delay(10);
    ReadTimeDate();
    Serial.print ("RTC Valid = "); Serial.println (RTC_Valid);
    if (RTC_Valid == 0)
    {
      Serial.println ("Preparing to set time and date.");
      SetTimeDate(18,8,16,12,0,0);
    }
    if (SerialStat) Serial.println (ReadTimeDate());
    digitalWrite (RTC_cs, HIGH);

//    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}
 
 //*************************************************************************************
 //*************************************************************************************
String ReadTimeDate()
{
  String temp;
  int kIndex;
  DateTime now = rtc.now();
  //DateTime nowUnix = rtc.unixtime();

  temp = "";
  temp.concat(now.unixtime());
  temp.concat(" ");
  temp.concat(now.year());
  temp.concat(".") ;
  if (now.month()<10) temp.concat ("0");
  temp.concat(now.month());
  temp.concat(".") ;
  if (now.day()<10) temp.concat ("0");
  temp.concat(now.day());
  temp.concat(" ") ;
  if (now.hour()<10) temp.concat ("0");
  temp.concat(now.hour());
  temp.concat(":") ;
  if (now.minute()<10) temp.concat ("0");
  temp.concat(now.minute());
  temp.concat(":") ;
  if (now.second()<10) temp.concat ("0");
  temp.concat(now.second());  

  //temp = temp.concat(now.year());
  return(temp);
  
}

 //*************************************************************************************
 //*************************************************************************************
void ShowHelp (void)
{
  Serial.println ("******************************************");
  Serial.println ("   Available Commands:");
  Serial.println ("     L List files");
  Serial.println ("     D Dump Current File");
  Serial.println ("     S Test the Serial (USB) connection");
  Serial.println ("     F Show file name");
  Serial.println ("     T");
  Serial.println ("     V");
  Serial.println ("     C");
  Serial.println ("******************************************");
  
}
/* ***************************************************************************************
 *     ***** SETUP *****
 ************************************************************************************** */
void setup() {
//rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  
  boolean n = 0;

 //*************************************************************************************
 // Start Serial port and wait for it to become active.
 Serial.begin(19200);
  for (int i=0; i<70; i++)
  {
    delay(50);
    n=!n;
    if (Serial) 
    { 
      SerialStat = true; i=70;
      digitalWrite (redLED, HIGH); 
    }
  }
  if (SerialStat) 
  {
    Serial.println ("\r\n============================================");
    Serial.println ("SPUNK-2020 Temperature/Orientation Logger");
   
  }
  pinMode(redLED, OUTPUT);
  pinMode (greenLED, OUTPUT);
  pinMode(temperature1Pin, INPUT);
  pinMode(temperature2Pin, INPUT);
  digitalWrite (greenLED, LOW); 
  digitalWrite (redLED, LOW);
  pinMode(blueLED, OUTPUT);

  if ( !rtc.begin())
  {
    Serial.println ("RTC not correct. Will reset.");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  pinMode (cardSelect, OUTPUT);
  digitalWrite (cardSelect, HIGH);
  
  if (SerialStat) Serial.println ("============================================");
  //----9DOF----------------------------------------------------------------------------------
imu.settings.device.commInterface = IMU_MODE_I2C;
imu.settings.device.mAddress = LSM9DS1_M;
imu.settings.device.agAddress = LSM9DS1_AG;
if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
  }
//------------------------------------------------------------------------------------------

 //*************************************************************************************
  // **************** Create an output file. ****************************

  CardExists = false;

  if (SerialStat) Serial.print ("SD card ");
  if (SD.begin(cardSelect))
  {
     if (SerialStat) Serial.println ("found");
     CardExists=true; 
     digitalWrite(greenLED, HIGH);
  }
  else
  {
    if (SerialStat) Serial.println ("not found.");
    CardExists = false;
  }

  int j;
  strcpy(filename, "SC_000.TXT");

  if (CardExists)
  {
    for (uint8_t i = 0; i < 1000; i++) 
    {
      j=i/100;
      filename[3] = '0' + j;
      j=i%100;
      filename[4] = '0' + j/10;
      j=j%10;
      filename[5] = '0' + j%10;
      // create if does not exist, do not open existing, write, sync after write
      if (! SD.exists(filename)) break;
    }
  
    logfile = SD.open(filename, FILE_WRITE);
    if( ! logfile ) 
    { SDStat = false;
      if (SerialStat) 
      {
        Serial.print("Couldn't create "); 
        Serial.println(filename);
        digitalWrite (greenLED, LOW);        
      }
    }
    else
    {
      SDStat = true;
      logfile.close();
      if (SerialStat) Serial.print("Writing to "); 
      if (SerialStat) Serial.println(filename);
    }
  }

  if (SerialStat) Serial.println ("============================================");

 //*************************************************************************************
 // Start i2c support
  Wire.begin();

 //*************************************************************************************

}


void printGyro()
{
  // Now we can use the gx, gy, and gz variables as we please.
  // Either print them as raw ADC values, or calculated in DPS.
  Serial.print("G: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcGyro helper function to convert a raw ADC value to
  // DPS. Give the function the value that you want to convert.
  Serial.print(imu.calcGyro(imu.gx), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gy), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gz), 2);
  Serial.println(" deg/s");
#elif defined PRINT_RAW
  Serial.print(imu.gx);
  Serial.print(", ");
  Serial.print(imu.gy);
  Serial.print(", ");
  Serial.println(imu.gz);
#endif
}

void printAccel()
{  
  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  Serial.print("A: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.
  Serial.print(imu.calcAccel(imu.ax), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.ay), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.az), 2);
  Serial.println(" g");
#elif defined PRINT_RAW 
  Serial.print(imu.ax);
  Serial.print(", ");
  Serial.print(imu.ay);
  Serial.print(", ");
  Serial.println(imu.az);
#endif

}

void printMag()
{  
  // Now we can use the mx, my, and mz variables as we please.
  // Either print them as raw ADC values, or calculated in Gauss.
  Serial.print("M: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcMag helper function to convert a raw ADC value to
  // Gauss. Give the function the value that you want to convert.
  Serial.print(imu.calcMag(imu.mx), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.my), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.mz), 2);
  Serial.println(" gauss");
#elif defined PRINT_RAW
  Serial.print(imu.mx);
  Serial.print(", ");
  Serial.print(imu.my);
  Serial.print(", ");
  Serial.println(imu.mz);
#endif
}

/* ******************************************************
 *  Loop.
 ***************************************************** */
long nt=0;
uint8_t i=0;
float t; float xVoltage; float xCurrent;
float v, temperature1, temperature2, temperature3;
int j;

void loop() {
  /* ***********************************************************
   * Grab the data, write to the SD card, write to the monitor.  
   */
//RTC_init();
DateTime now = rtc.now();
  t=nt; t=t/1000.;  //nt counts milliseconds for timing.
 if (millis() >= nt)
 {  
  xVoltage = 0; xCurrent = 0;
  for (j=0; j<10; j++)
  {
    //Serial.print ('*');
   // ReadCurrentMonitor();
    //Serial.print ('&');
    //iVoltage=1; iShunt = 1;
    xVoltage = xVoltage + iVoltage;
    if (iShunt > 32767) iShunt = iShunt - 65536;
    xCurrent = xCurrent + iShunt;
    delay(10);  
  }
  
  xVoltage = (xVoltage/20000.) -.008; //A small calibration
  xCurrent = xCurrent / 100.;

  v=0;
  temperature1 = 0;
  temperature2 = 0;
  for (j = 0; j<100; j++)
    {
      v = v + analogRead (vBatPin);
      temperature1 = temperature1 + analogRead(temperature1Pin);
      temperature2 = temperature2 + analogRead(temperature2Pin);
      temperature3 = temperature3 + analogRead(temperature3Pin);
    }
  
  v = v * .66 / 1024.;
  //temperature1 = sqrt(((1.8639 - (temperature1/10./1024*3.3))/3.88*1000000) + 2196200) - 1481.96; // LM19 temp. sensor
  temperature2 = sqrt(((1.8639 - (temperature2/100./1024*3.3))/3.88*1000000) + 2196200) - 1481.96; // LM19 temp. sensor
  temperature3 = sqrt(((1.8639 - (temperature3/100./1024*3.3))/3.88*1000000) + 2196200) - 1481.96; // LM19 temp. sensor

temperature1=temperature1/100;
//temperature2=temperature2*3.3/10/1024;

    if ( imu.gyroAvailable() )
  {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu.readGyro();
  }
  if ( imu.accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
  }
  if ( imu.magAvailable() )
  {
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu.readMag();
  }
  
  if (SerialStat)   //Save time and date.
  { digitalWrite (cardSelect, HIGH);  digitalWrite (RTC_cs, HIGH); //Just to be sure
  
    digitalWrite(redLED, HIGH);
    Serial.print (ReadTimeDate()); Serial.print (" ");
  // Serial.print (now.unixtime()); Serial.print (" "); 
    Serial.print (t);  Serial.print (" "); 
    Serial.print (v,2); Serial.print (" ");
    Serial.print (temperature1,2); Serial.print (" ");
    Serial.print (temperature2,2); Serial.print (" ");
    Serial.print (temperature3,2); Serial.print (" ");
    Serial.print(imu.calcAccel(imu.ax), 2);Serial.print(" ");
    Serial.print(imu.calcAccel(imu.ay), 2);Serial.print(" ");
    Serial.print(imu.calcAccel(imu.az), 2);Serial.print(" ");
    Serial.print(imu.calcGyro(imu.gx), 2);Serial.print(" ");
    Serial.print(imu.calcGyro(imu.gy), 2);Serial.print(" ");
    Serial.print(imu.calcGyro(imu.gz), 2); Serial.print(" ");
    Serial.print(imu.calcMag(imu.mx), 2);  Serial.print(" ");
    Serial.print(imu.calcMag(imu.my), 2);  Serial.print(" ");
    Serial.println(imu.calcMag(imu.mz), 2); // Serial.print(" ");
     
    //Serial.print (xVoltage,2);Serial.print (", "); Serial.println (xCurrent,1);
    //digitalWrite (redLED, LOW);
   // printGyro();  // Print "G: gx, gy, gz"
   // printAccel(); // Print "A: ax, ay, az"
   // printMag();   // Print "M: mx, my, mz"
  // Serial.println(imu.calcGyro(imu.gx), 2);
  }

  if (SDStat)
  {
    digitalWrite(blueLED, HIGH);
    Serial.print ("*");
    logfile = SD.open(filename, FILE_WRITE);
    logfile.print (ReadTimeDate()); 
    logfile.print (" "); 
    logfile.print (t); logfile.print (" "); 
    logfile.print (v,2); logfile.print (" ");
    logfile.print (temperature1,2); logfile.print (" ");
    logfile.print (temperature2,2); logfile.print (" ");
    logfile.print (temperature3,2); logfile.print (" ");
    logfile.print(imu.calcAccel(imu.ax), 2);  logfile.print(" ");
    logfile.print(imu.calcAccel(imu.ay), 2);  logfile.print(" ");
    logfile.print(imu.calcAccel(imu.az), 2);  logfile.print(" ");
    logfile.print(imu.calcGyro(imu.gx), 2);   logfile.print(" ");
    logfile.print(imu.calcGyro(imu.gy), 2);   logfile.print(" ");
    logfile.print(imu.calcGyro(imu.gz), 2);   logfile.print(" ");
    logfile.print(imu.calcMag(imu.mx), 2);    logfile.print(" ");
    logfile.print(imu.calcMag(imu.my), 2);    logfile.print(" ");
    logfile.println(imu.calcMag(imu.mz), 2); // Serial.print(" ");


    
    //logfile.print(xVoltage,2); logfile.print (", "); logfile.println (xCurrent,1);
    logfile.flush();
    logfile.close();
    //digitalWrite(greenLED, LOW);
  }
  
  delay(50);
  digitalWrite(blueLED, LOW);
  digitalWrite(greenLED,LOW);
  
  //Update time to the next acq.
  nt=nt+500;
 }

   if(Serial.available()) 
    {
     // char inChar = (char)Serial.read();
     String incoming = "";
      while (Serial.available())
      {
         incoming = Serial.readString();
    
      if ((incoming[0] == 's') && (incoming[1] == 't'))
        {
         Serial.println("yowza"); 
         String STyear = "";
         String STmonth = "";
         String STday = "";
         String SThour = "";
         String STminute = "";
         String STsecond = "";
         STyear = STyear + incoming[2] + incoming[3]+incoming[4]+incoming[5];
         int STyearInt = STyear.toInt();
         
         STmonth = STmonth + incoming[6] + incoming[7];
         int STmonthInt = STmonth.toInt();

         STday = STday + incoming[8] + incoming[9];
         int STdayInt = STday.toInt();

         SThour = SThour + incoming[10] + incoming[11];
         int SThourInt = SThour.toInt();

         STminute = STminute + incoming[12] + incoming[13];
         int STminuteInt = STminute.toInt();

         STsecond = STsecond + incoming[14] + incoming[15];
         int STsecondInt = STsecond.toInt();
         
         
         //int STyear = incoming[3],.toInt())
         Serial.println(STyearInt); 
         rtc.adjust(DateTime(STyearInt, STmonthInt, STdayInt, SThourInt, STminuteInt, STsecondInt));
        }
        
      }
      
      
    //  char iD = (char)Serial.read();
    //  if (inChar != '\n') 
    //  {
       
       
    //  }
      
    }
  //while (millis()<nt){};
}
