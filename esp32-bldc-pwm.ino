
#define P0_LIN 25
#define P0_HIN 26
#define P1_LIN 32
#define P1_HIN 33
#define P2_LIN 14
#define P2_HIN 12

#define P_FLOAT 0
#define P_HI 1
#define P_LO 2

#define INDEX_CHANGE_MS 1

#define EYE_PIN 27

HardwareSerial &Debug = Serial;

const char *compileDate = __DATE__;
const char *compileTime = __TIME__;

const int commPattern[6][3] = {
    {P_HI, P_LO, P_FLOAT},
    {P_HI, P_FLOAT, P_LO},
    {P_FLOAT, P_HI, P_LO},
    {P_LO, P_HI, P_FLOAT},
    {P_LO, P_FLOAT, P_HI},
    {P_FLOAT, P_LO, P_HI}};
int idx = 0;

int testLIN = P0_LIN;
int testHIN = P0_HIN;

boolean ledOn = false;

volatile int interruptCounter;
int totalInterruptCounter;
int totalCycleCounter;

int cycleResetSlow = 1000;
int cycleResetFast = 150;
int cycleReset;
int accelerateCycles = 30000;
int steadyCycles = 100000;
bool accel = true;
bool decel = false;
int accelCounter;
int accelCountPerCycleReset;

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer();
void setPhaseState(int lin, int hin, int state);
void setCommIndex(int commIdx);

volatile int encoderCounter;
long totalEncoderCount;
long lastReportMillis;
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

void attachInts();
void detachInts();
void IRAM_ATTR onEncoder();

void setup()
{
    Debug.begin(115200);
    Debug.println("Booting");
    Debug.print("Compile date: ");
    Debug.println(compileDate);
    Debug.print("Compile time: ");
    Debug.println(compileTime);

    interruptCounter = 0;
    totalInterruptCounter = 0;
    totalCycleCounter = 0;
    accelCounter = 0;

    accelCountPerCycleReset = accelerateCycles / (cycleResetSlow - cycleResetFast);
    Debug.printf("Accel count: %d\n", accelCountPerCycleReset);

    cycleReset = cycleResetSlow;

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);

    pinMode(P0_LIN, OUTPUT);
    pinMode(P0_HIN, OUTPUT);
    ledcSetup(0, 50000, 8);
    ledcAttachPin(P0_HIN, 0);
    pinMode(P1_LIN, OUTPUT);
    pinMode(P1_HIN, OUTPUT);
    ledcSetup(1, 50000, 8);
    ledcAttachPin(P1_HIN, 1);
    pinMode(P2_LIN, OUTPUT);
    pinMode(P2_HIN, OUTPUT);
    ledcSetup(2, 50000, 8);
    ledcAttachPin(P2_HIN, 2);

    digitalWrite(P0_LIN, HIGH);
    ledcWrite(0, 0);
    digitalWrite(P1_LIN, HIGH);
    ledcWrite(1, 0);
    digitalWrite(P2_LIN, HIGH);
    ledcWrite(2, 0);

    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 30000, true);
    timerAlarmEnable(timer);

    encoderCounter = 0;
    totalEncoderCount = 0;
    lastReportMillis = 0;
    attachInts();

    setCommIndex(idx);
}

void loop()
{
    if (millis() > (lastReportMillis + 500))
    {
        lastReportMillis = millis();

        Debug.printf("Encoder count: %ld\n", totalEncoderCount);
    }

    if (encoderCounter > 0)
    {
        portENTER_CRITICAL(&encoderMux);
        encoderCounter--;
        portEXIT_CRITICAL(&encoderMux);

        totalEncoderCount++;
    }

    if (interruptCounter > 0)
    {
        portENTER_CRITICAL(&timerMux);
        interruptCounter--;
        portEXIT_CRITICAL(&timerMux);

        totalInterruptCounter++;

        // if (totalCycleCounter >= (int)cycleReset)
        // {
        idx++;
        if (idx > 5)
        {
            idx = 0;
        }

        setCommIndex(idx);

        ledOn = !ledOn;
        digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
    }
}

void IRAM_ATTR onTimer()
{
    portENTER_CRITICAL_ISR(&timerMux);
    interruptCounter++;
    portEXIT_CRITICAL_ISR(&timerMux);
}

void setPhaseState(int lin, int hin, int state)
{
    switch (state)
    {
    case P_FLOAT:
        // Float
        // Debug.println(lin + " Float");
        digitalWrite(lin, LOW);
        ledcWrite(hin, 0);
        // digitalWrite(hin, LOW);
        break;
    case P_HI:
        // HI
        // Debug.println(lin + " HI");
        digitalWrite(lin, LOW);
        ledcWrite(hin, 127);
        // digitalWrite(hin, HIGH);
        break;
    case P_LO:
        // LO
        // Debug.println(lin + " LO");
        ledcWrite(hin, 0);
        // digitalWrite(hin, LOW);
        digitalWrite(lin, HIGH);
        break;
    }
}

void setCommIndex(int commIdx)
{
    setPhaseState(P0_LIN, 0, commPattern[commIdx][0]);
    setPhaseState(P1_LIN, 1, commPattern[commIdx][1]);
    setPhaseState(P2_LIN, 2, commPattern[commIdx][2]);
}

void IRAM_ATTR onEncoder()
{
    portENTER_CRITICAL_ISR(&encoderMux);
    encoderCounter++;
    portEXIT_CRITICAL_ISR(&encoderMux);
}

void attachInts()
{
    attachInterrupt(digitalPinToInterrupt(EYE_PIN), onEncoder, FALLING);
}

void detachInts()
{
    detachInterrupt(digitalPinToInterrupt(EYE_PIN));
}