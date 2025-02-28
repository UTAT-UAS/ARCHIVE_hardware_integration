// test esp32 rotary encoder library
#include <Arduino.h>
#define DIR_NONE 0x00           // No complete step yet.
#define DIR_CW   0x10           // Clockwise step.
#define DIR_CCW  0x20           // Anti-clockwise step.
#define R_START     0x3
#define R_CW_BEGIN  0x1
#define R_CW_NEXT   0x0
#define R_CW_FINAL  0x2
#define R_CCW_BEGIN 0x6
#define R_CCW_NEXT  0x4
#define R_CCW_FINAL 0x5

const unsigned char ttable[8][4] = {
    {R_CW_NEXT,  R_CW_BEGIN,  R_CW_FINAL,  R_START},                // R_CW_NEXT
    {R_CW_NEXT,  R_CW_BEGIN,  R_CW_BEGIN,  R_START},                // R_CW_BEGIN
    {R_CW_NEXT,  R_CW_FINAL,  R_CW_FINAL,  R_START | DIR_CW},       // R_CW_FINAL
    {R_START,    R_CW_BEGIN,  R_CCW_BEGIN, R_START},                // R_START
    {R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START},                // R_CCW_NEXT
    {R_CCW_NEXT, R_CCW_FINAL, R_CCW_FINAL, R_START | DIR_CCW},      // R_CCW_FINAL
    {R_CCW_NEXT, R_CCW_BEGIN, R_CCW_BEGIN, R_START},                // R_CCW_BEGIN
    {R_START,    R_START,     R_START,     R_START}                 // ILLEGAL
};

class Test 
{
public:
    Test();
    ~Test() = default;
    int GetEncoderLen();
    void EncoderSetup();

    static void IRAM_ATTR EncoderISR();

private:
    void IRAM_ATTR ReadEncoder();
    int pinA_ = 2;
    int pinB_ = 4;
    volatile int encoderRaw_{0}; // read from encoders
    float encoderLen_{0};
    float lastEncoderLen_{0};
    unsigned long lastEncoderChangeTime_{0};
    static Test *instance_;

    // encoder state
    unsigned char encState_{0};

};

Test::Test()
{
    instance_ = this;
}
 

int Test::GetEncoderLen()
{
    noInterrupts();
    int temp = encoderRaw_;
    interrupts();
    return temp;
}

void Test::EncoderSetup()
{
    pinMode(pinA_, INPUT_PULLUP);
    pinMode(pinB_, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(2), EncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(4), EncoderISR, CHANGE);
}
void IRAM_ATTR Test::EncoderISR()
{
    instance_->ReadEncoder();
    //Serial.println("ISR triggered");
}

void IRAM_ATTR Test::ReadEncoder()
{
    const unsigned char state = (digitalRead(pinA_) << 1) | digitalRead(pinB_);
    encState_ = ttable[encState_ & 0x07][state];
    
    if (encState_ & DIR_CW) {
        encoderRaw_++;
    } else if (encState_ & DIR_CCW) {
        encoderRaw_--;
    }
}

Test *Test::instance_ = nullptr;
Test test;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    test.EncoderSetup();
    Serial.println("Encoder setup");
}

void loop() {
    // put your main code here, to run repeatedly:
    //Serial.print("Encoder Len: ");
    Serial.println(test.GetEncoderLen());
    delay(100);
}

