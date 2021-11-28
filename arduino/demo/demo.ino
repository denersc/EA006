#define SYNC_BYTE 0xC7
#define N_SENSORS 2

typedef struct _ChCtx
{
  byte analogPin;
  int chargePin;
  int dischargePin;
  double capValue;
  double resValue;
} ChCtx;

typedef struct _SensorData {
  size_t nData;
  char* data;
} SensorData;

ChCtx ch[4];
SensorData *sd;
const byte analogPins[4] = {A0, A1, A2, A3};
const int chargePins[4] = {9, 10, 11, 12};
const int dischargePins[4] = {5, 6, 7, 8}; 
const double capValues[4] = {10, 1, 1, 1};
unsigned long startTime;
unsigned long elapsedTime;
double ohms;

/* SensorData Functions */
SensorData *SDCreate(size_t len, uint16_t sync)
{
  SensorData* sd = (SensorData*)malloc(sizeof(SensorData));
  sd->data = (char*)malloc(((len*2)+1)*sizeof(char));
  sd->nData = len;
  sd->data[0] = SYNC_BYTE;
  return sd;
}

void SDAddDataToIndex(SensorData *sd, uint16_t data, size_t index)
{
  sd->data[index*2+1] = data >> 8;
  sd->data[index*2+2] = data & 0xFF;
}

char *SDGetData(SensorData *sd)
{
  return sd->data;
}

size_t SDGetLength(SensorData *sd)
{
  return sd->nData*2 + 1;
}

void setup(){
  for(int i = 0; i < 2; i++)
  {
    ch[i].analogPin = analogPins[i];
    ch[i].chargePin = chargePins[i];
    ch[i].dischargePin = dischargePins[i];
    ch[i].capValue = capValues[i];
    pinMode(chargePins[i], OUTPUT);
    digitalWrite(chargePins[i], LOW);
  }
  sd = SDCreate(2, SYNC_BYTE);
  Serial.begin(115200);
}

void loop(){
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(ch[i].chargePin, HIGH);
    startTime = micros();
    int reading;
    while((reading = analogRead(ch[i].analogPin)) < 648) {}
    elapsedTime = micros() - startTime;
    ch[i].resValue = ((double)elapsedTime / (ch[i].capValue));
    SDAddDataToIndex(sd, (uint16_t)ch[i].resValue, i);
    digitalWrite(ch[i].chargePin, LOW);
    pinMode(ch[i].dischargePin, OUTPUT);
    digitalWrite(ch[i].dischargePin, LOW);
  }

  Serial.write(SDGetData(sd), SDGetLength(sd));
  for (int i = 0; i < 2; i++)
  {
    while(analogRead(ch[i].analogPin > 0)) {}
    pinMode(ch[i].dischargePin, INPUT);
  }


}
