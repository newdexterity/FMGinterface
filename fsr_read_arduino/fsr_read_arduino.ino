#define FSR0           A0
#define FSR1           A1
#define FSR2           A2
#define FSR3           A3
#define FSR4           A4
#define FSR5           A5


int FSRReading_0 = 0;
int FSRReading_1 = 0;
int FSRReading_2 = 0;
int FSRReading_3 = 0;
int FSRReading_4 = 0;
int FSRReading_5 = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
//  FSRReadingRand(); // Delete Rand to view actual FSR readings
  FSRReading(); // Delete Rand to view actual FSR readings
  plotFSRValues();
}


void FSRReading()
{
  FSRReading_0 = analogRead(FSR0);
  FSRReading_1 = analogRead(FSR1);
  FSRReading_2 = analogRead(FSR2);
  FSRReading_3 = analogRead(FSR3);
  FSRReading_4 = analogRead(FSR4);
  FSRReading_5 = analogRead(FSR5);
}

void FSRReadingRand()
{
  FSRReading_0 = random(0,100);
  FSRReading_1 = random(150,250);
  FSRReading_2 = random(300,400);
  FSRReading_3 = random(450,550);
  FSRReading_4 = random(600,700);
  FSRReading_5 = random(750,850);
}

void plotFSRValues()
{
  // Set upper and lower limits for plot
  Serial.print(0);
  Serial.print(' ');
  Serial.print(1200);
  
  // FSR 0
  Serial.print(' ');
  Serial.print(FSRReading_0);

  // FSR 1
  Serial.print(' ');
  Serial.print(FSRReading_1);

  // FSR 2
  Serial.print(' ');
  Serial.print(FSRReading_2);

  // FSR 3
  Serial.print(' ');
  Serial.print(FSRReading_3);

  // FSR 4
  Serial.print(' ');
  Serial.print(FSRReading_4);

  // FSR 5
  Serial.print(' ');
  Serial.println(FSRReading_5);

  // End line for serial plotter
  //Serial.println(' ');
  
}
