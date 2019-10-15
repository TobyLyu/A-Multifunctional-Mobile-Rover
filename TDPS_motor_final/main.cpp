
#include "mbed.h"
#include "math.h"

PwmOut motor1(PB_1);//Define PWM to control left wheel
PwmOut motor2(PB_4);//PWM control right wheel
PwmOut motor3(PB_0);
PwmOut motor4(PB_5);

InterruptIn LF_C(PB_15);//encoder of left wheel
InterruptIn LB_C(PA_10);//encoder of right wheel
InterruptIn RF_C(PA_4);//encoder of right wheel
InterruptIn RB_C(PB_8);//encoder of right wheel

DigitalOut LF_DIR(PB_14);//0
DigitalOut LB_DIR(PA_11);//0
DigitalOut RF_DIR(PA_0);//1
DigitalOut RB_DIR(PB_6);//1

DigitalOut LF_EN(PB_13);//1
DigitalOut LB_EN(PA_12);//1
DigitalOut RF_EN(PA_1);//1
DigitalOut RB_EN(PB_7);//1    PA_13,PA_14 do not use!!!!!!!

DigitalOut output(PA_15);       //rotate
DigitalIn input1(PB_11);      //feed
DigitalIn input(PB_12);         //input of action

Ticker t;
Timer rotate_time;
Serial async_port(PA_2, PA_3); //set up TX and RX on pins 9 and 10


float  rate=0.05;

float  PWMLF=0;
float  PWMLB=0;
float  PWMRF=0;
float  PWMRB=0;
float  preaa=0;
float  prebb=0;
float  precc=0;
float  predd=0;
float  errora=0;
float  errorb=0;
float  errorc=0;
float  errord=0;
float  prea=0;
float  preb=0;
float  prec=0;
float  pred=0;
int  PREA=0;
int  PREB=0;
int  PREC=0;
int  PRED=0;
unsigned int a = 0;//initialize counts of the left encoding wheel
unsigned int b = 0;//initialize counts of the right encoding wheel
unsigned int c = 0;
unsigned int d = 0;
float seta = 0.35;
float setb = 0.35;
float setc = 0.35;
float setd = 0.35;
float move_length = 0.0;
int ang = 90;


float min(float x, float y)
{
    return (x<y)?x:y;
}

float rotatetime = 0.0;

void reset_PID(void)
{
    PWMLF=0;
    PWMLB=0;
    PWMRF=0;
    PWMRB=0;
    preaa=0;
    prebb=0;
    precc=0;
    predd=0;
    errora=0;
    errorb=0;
    errorc=0;
    errord=0;
    prea=0;
    preb=0;
    prec=0;
    pred=0;
    PREA=0;
    PREB=0;
    PREC=0;
    PRED=0;
    a = 0;//initialize counts of the left encoding wheel
    b = 0;//initialize counts of the right encoding wheel
    c = 0;
    d = 0;
    seta = 0.35;
    setb = 0.35;
    setc = 0.35;
    setd = 0.35;
    move_length = 0.0;
}

void c_LF()                                             //this is for encode wheel
{
    a +=1;
}
void c_LB()
{
    b +=1;
}
void c_RF()
{
    c +=1;
}
void c_RB()
{
    d +=1;
}

float speed(int count, int previous)                    //calculate the current speed
{
    return (count+previous)*0.41f/(40320.0f*rate*2);
}

float SetleftF(float e1, float e2, float e3)
{
    return 0.92f*(e1-e2)+0.272f*e1+0.6f*(e1-2*e2+e3);
}
float SetleftB(float e1,float e2, float e3)
{
    return 0.89f*(e1-e2)+0.316f*e1+0.6f*(e1-2*e2+e3);
}
float SetrightF(float e1,float e2, float e3)
{
    return 0.88f*(e1-e2)+0.298f*e1+0.6f*(e1-2*e2+e3);
}
float SetrightB(float e1,float e2, float e3)
{
    return 0.89f*(e1-e2)+0.262f*e1+0.6f*(e1-2*e2+e3);
}

void PID(void)                                          //PID program
{
    float SPA = speed(a, PREA);
    float SPB = speed(b, PREB);
    float SPC = speed(c, PREC);
    float SPD = speed(d, PRED);
    PREA = a;
    PREB = b;
    PREC = c;
    PRED = d;
    move_length = ((a+b+c+d)*0.41f)/(4.0f*40320.0f)+ move_length;
    a = 0;
    b = 0;
    c = 0;
    d = 0;
    errora=seta-SPA;
    errorb=setb-SPB;
    errorc=setc-SPC;
    errord=setd-SPD;

    PWMLF=PWMLF+SetleftF(errora,prea,preaa);
    PWMLB=PWMLB+SetleftB(errorb,preb,prebb);
    PWMRF=PWMRF+SetrightF(errorc,prec,precc);
    PWMRB=PWMRB+SetrightB(errord,pred,predd);

    preaa=prea; //Get integral term right
    prea=errora;
    prebb=preb; //Get integral term right
    preb=errorb;
    precc=prec; //Get integral term right
    prec=errorc;
    predd=pred; //Get integral term right
    pred=errord;

    if(PWMLF>0.9f)
        PWMLF=0.9;
    else if(PWMLF<0.2f)
        PWMLF=0.2;
    if(PWMRF>0.9f)
        PWMRF=0.9;
    else if(PWMRF<0.2f)
        PWMRF=0.2;
    if(PWMLB>0.9f)
        PWMLB=0.9;
    else if(PWMLB<0.2f)
        PWMLB=0.2;
    if(PWMRB>0.9f)
        PWMRB=0.9;
    else if(PWMRB<0.2f)
        PWMRB=0.2;

    motor1.pulsewidth_us(100.0f*PWMLF);
    motor2.pulsewidth_us(100.0f*PWMLB);
    motor3.pulsewidth_us(100.0f*PWMRF);
    motor4.pulsewidth_us(100.0f*PWMRB);//here PWMRB should be the variable if the wheel reading is correct.
}

void go(float speedL, float speedR, float length)       //go straight(left speed, right speed, distance)
{
    reset_PID();
    if(length<0) {
        motor1.pulsewidth_us(0);
        motor2.pulsewidth_us(0);
        motor3.pulsewidth_us(0);
        motor4.pulsewidth_us(0);
        wait_ms(500);
        LF_DIR=1;
        LB_DIR=1;
        RF_DIR=0;
        RB_DIR=0;
    }
    if(length>0) {
        motor1.pulsewidth_us(0);
        motor2.pulsewidth_us(0);
        motor3.pulsewidth_us(0);
        motor4.pulsewidth_us(0);
        wait_ms(500);
        LF_DIR=0;
        LB_DIR=0;
        RF_DIR=1;
        RB_DIR=1;
    }
    seta = speedL;
    setb = speedL;
    setc = speedR;
    setd = speedR;
    wait_ms(500);                 //wait for at least 1ms for direction being set
    t.attach(&PID,rate);        //start PID (robot move from now)
    while (move_length < abs(length)) {
        wait_ms(1);
    }   //the robot will be totally controlled by PID
    t.detach();                 //stop PID to get control of the robot
    motor1.pulsewidth_us(0);    //stop the robot
    motor2.pulsewidth_us(0);
    motor3.pulsewidth_us(0);
    motor4.pulsewidth_us(0);
    move_length = 0.0;          //reset the moving length
}

void feed(float speedL, float speedR, float length)       //go straight(left speed, right speed, distance)
{
    reset_PID();
    if(length<0) {
        motor1.pulsewidth_us(0);
        motor2.pulsewidth_us(0);
        motor3.pulsewidth_us(0);
        motor4.pulsewidth_us(0);
        wait_ms(500);
        LF_DIR=1;
        LB_DIR=1;
        RF_DIR=0;
        RB_DIR=0;
    }
    if(length>0) {
        motor1.pulsewidth_us(0);
        motor2.pulsewidth_us(0);
        motor3.pulsewidth_us(0);
        motor4.pulsewidth_us(0);
        wait_ms(500);
        LF_DIR=0;
        LB_DIR=0;
        RF_DIR=1;
        RB_DIR=1;
    }
    seta = speedL;
    setb = speedL;
    setc = speedR;
    setd = speedR;
    wait_ms(500);                 //wait for at least 1ms for direction being set
    t.attach(&PID,rate);        //start PID (robot move from now)
    output = 1;
    while ((move_length < abs(length))&&input.read()==0) {
        wait_ms(1);
    }   //the robot will be totally controlled by PID
    t.detach();                 //stop PID to get control of the robot
    output = 0;
    motor1.pulsewidth_us(0);    //stop the robot
    motor2.pulsewidth_us(0);
    motor3.pulsewidth_us(0);
    motor4.pulsewidth_us(0);
    move_length = 0.0;          //reset the moving length
    while(input.read()) {
        wait_ms(1);
    }
}

/*void rotate(float sp, signed int angle){//speed, degree (this function is not completed)
    move_length = 0.0;
    a = 0;  b = 0; c = 0; d = 0;
    PREA = 0; PREB = 0; PREC = 0; PRED = 0;
    int times = abs(angle)/45;
    //float length = (2.0f*0.1386768f*3.1415f*1.88f*abs(angle))/360.0f;     //calculate the moving length
    if(angle<0){
        motor1.pulsewidth_us(0);
        motor2.pulsewidth_us(0);
        motor3.pulsewidth_us(0);
        motor4.pulsewidth_us(0);
        wait_ms(500);
        LF_DIR=1;
        LB_DIR=1;
        RF_DIR=1;
        RB_DIR=1;
        }
    if(angle>0){
        motor1.pulsewidth_us(0);
        motor2.pulsewidth_us(0);
        motor3.pulsewidth_us(0);
        motor4.pulsewidth_us(0);
        wait_ms(500);
        LF_DIR=0;
        LB_DIR=0;
        RF_DIR=0;
        RB_DIR=0;
        }
    output = 1;
    for(int i = 0;i<times;i++){                 //wait for at least 1ms for direction being set
    wait(2);
    rotate_time.start();
    while (!input.read()){
        wait_ms(1);
        motor1.pulsewidth_us(sp*100);
        motor2.pulsewidth_us(sp*100);
        motor3.pulsewidth_us(sp*100);
        motor4.pulsewidth_us(sp*100);
        }
    rotate_time.stop();
    rotatetime = rotate_time.read();
    rotate_time.reset();

    motor1.pulsewidth_us(0);
    motor2.pulsewidth_us(0);
    motor3.pulsewidth_us(0);
    motor4.pulsewidth_us(0);
    }
    output = 0;
    }*/

float compare(float P)
{
    P = (P<90)?P:90;
    P = (P>15)?P:15;
    return P;
}

void rotate(float sp, signed int angle) //speed, degree (this function is not completed)
{
    //int times = abs(angle)/90;
    wait(3);
    reset_PID();
    a = 0;
    b = 0;
    c = 0;
    d = 0;
    float length = (2.0f*0.1386768f*3.1415f*1.88f*abs(angle))/360.0f;     //calculate the moving length
    if(angle<0) {
        motor1.pulsewidth_us(0);
        motor2.pulsewidth_us(0);
        motor3.pulsewidth_us(0);
        motor4.pulsewidth_us(0);
        wait_ms(1);
        LF_DIR=1;
        LB_DIR=1;
        RF_DIR=1;
        RB_DIR=1;
    }
    if(angle>0) {
        motor1.pulsewidth_us(0);
        motor2.pulsewidth_us(0);
        motor3.pulsewidth_us(0);
        motor4.pulsewidth_us(0);
        wait_ms(1);
        LF_DIR=0;
        LB_DIR=0;
        RF_DIR=0;
        RB_DIR=0;
    }
    wait_ms(1);                 //wait for at least 1ms for direction being set
    while (move_length < abs(length)) {
        move_length = ((a+b+c+d)*0.41f)/(4.0f*40320.0f)+ move_length;
        a = 0;
        b = 0;
        c = 0;
        d = 0;
        motor1.pulsewidth_us(sp*100);    //stop the robot
        motor2.pulsewidth_us(sp*100);
        motor3.pulsewidth_us(sp*100);
        motor4.pulsewidth_us(sp*100);
    }   //the robot will be totally controlled by PID
    motor1.pulsewidth_us(0);    //stop the robot
    motor2.pulsewidth_us(0);
    motor3.pulsewidth_us(0);
    motor4.pulsewidth_us(0);
    move_length = 0.0;          //reset the moving length
}

void find(void)
{
    move_length = ((a+b+c+d)*0.41f)/(4.0f*40320.0f)+ move_length;
    a = 0;
    b = 0;
    c = 0;
    d = 0;
    if(180>ang&&ang>120) {              //sharp turn left
        motor1.pulsewidth_us(0);
        motor2.pulsewidth_us(0);
        motor3.pulsewidth_us(0);
        motor4.pulsewidth_us(0);
        wait_ms(1);
        LF_DIR=1;
        LB_DIR=1;
        RF_DIR=0;
        RB_DIR=0;
        wait_ms(1);
        PWMLF=25+(ang-90)*1;
        PWMLB=25+(ang-90)*1;
        PWMRF=25-(ang-90)*1;
        PWMRB=25-(ang-90)*1;
        PWMLF = compare(PWMLF);
        PWMLB = compare(PWMLB);
        PWMRF = compare(PWMRF);
        PWMRB = compare(PWMRB);
        motor1.pulsewidth_us(PWMLF);
        motor2.pulsewidth_us(PWMLB);
        motor3.pulsewidth_us(PWMRF);
        motor4.pulsewidth_us(PWMRB);
    } else if(0<ang&&ang<60) {          //sharp turn right
        motor1.pulsewidth_us(0);
        motor2.pulsewidth_us(0);
        motor3.pulsewidth_us(0);
        motor4.pulsewidth_us(0);
        wait_ms(1);
        LF_DIR=1;
        LB_DIR=1;
        RF_DIR=0;
        RB_DIR=0;
        wait_ms(1);
        PWMLF=25+(ang-90)*1;
        PWMLB=25+(ang-90)*1;
        PWMRF=25-(ang-90)*1;
        PWMRB=25-(ang-90)*1;
        PWMLF = compare(PWMLF);
        PWMLB = compare(PWMLB);
        PWMRF = compare(PWMRF);
        PWMRB = compare(PWMRB);
        motor1.pulsewidth_us(PWMLF);
        motor2.pulsewidth_us(PWMLB);
        motor3.pulsewidth_us(PWMRF);
        motor4.pulsewidth_us(PWMRB);
    } else if(60<=ang&&ang<=120) {      //slightly adjust
        motor1.pulsewidth_us(0);
        motor2.pulsewidth_us(0);
        motor3.pulsewidth_us(0);
        motor4.pulsewidth_us(0);
        wait_ms(1);
        LF_DIR=1;
        LB_DIR=1;
        RF_DIR=0;
        RB_DIR=0;
        wait_ms(1);
        PWMLF=25+(ang-90);
        PWMLB=25+(ang-90);
        PWMRF=25-(ang-90);
        PWMRB=25-(ang-90);
        PWMLF = compare(PWMLF);
        PWMLB = compare(PWMLB);
        PWMRF = compare(PWMRF);
        PWMRB = compare(PWMRB);
        motor1.pulsewidth_us(PWMLF);
        motor2.pulsewidth_us(PWMLB);
        motor3.pulsewidth_us(PWMRF);
        motor4.pulsewidth_us(PWMRB);
    }

}

void openmv(void)
{
    a = 0;
    b = 0;
    c = 0;
    d = 0;
    move_length = 0;
    async_port.putc('5');                   //start feeding process
    wait(2);
    while(1) {
        move_length = ((a+b+c+d)*0.41f)/(4.0f*40320.0f)+ move_length;
        a = 0;
        b = 0;
        c = 0;
        d = 0;
        while(async_port.readable()==1) { //is there a character to be read?
            ang=async_port.getc();
        } //if yes, t
        if(ang==240 || move_length > 11.5) {
            async_port.putc('8');
            break;
        }
        find();
        wait_ms(100);
    }
    motor1.pulsewidth_us(0);
    motor2.pulsewidth_us(0);
    motor3.pulsewidth_us(0);
    motor4.pulsewidth_us(0);
}

void openmv1(void)
{
    a = 0;
    b = 0;
    c = 0;
    d = 0;
    move_length = 0;
    async_port.putc('6');                   //start feeding process
    wait(2);
    while(1) {
        move_length = ((a+b+c+d)*0.41f)/(4.0f*40320.0f)+ move_length;
        a = 0;
        b = 0;
        c = 0;
        d = 0;
        while(async_port.readable()==1) { //is there a character to be read?
            ang=async_port.getc();
        } //if yes, t
        if(ang==241 || move_length > 13.5) {
            async_port.putc('8');
            break;
        }
        find();
        wait_ms(100);
    }
    motor1.pulsewidth_us(0);
    motor2.pulsewidth_us(0);
    motor3.pulsewidth_us(0);
    motor4.pulsewidth_us(0);
}

int main()
{
    LF_EN=1;
    LB_EN=1;
    RF_EN=1;
    RB_EN=1;
    LF_DIR=0;
    LB_DIR=0;
    RF_DIR=1;
    RB_DIR=1;
    output = 0;
    input.mode(PullDown);

    // Set PWM
    motor1.period_us(100);//initialize left motor
    LF_C.rise(&c_LF);           //set interrupt mode
    LF_C.mode(PullDown);
    LB_C.rise(&c_LB);
    LB_C.mode(PullDown);
    RF_C.rise(&c_RF);
    RF_C.mode(PullDown);
    RB_C.rise(&c_RB);
    RB_C.mode(PullDown);
    async_port.baud(19200);
    async_port.format(8, Serial::None,1);
    wait(6);

    go(0.3,0.3,2.50f);                      //move to color pad
    rotate(0.30,-57);
    go(0.3,0.3,0.68f);
    wait(1);
    output = 1;                             //start color detect
    while(!input.read()&&!input1.read()) {                  //wait for color result
        wait_ms(1);
    }
    bool color1 = input1.read();
    bool color = input.read();
    output = 0;
    wait(1);

    go(0.3,0.3,2.16f);
    if(color1 == 0&&color == 1) {          //red
        rotate(0.30,57);
        wait(1);
        go(0.3,0.3,2.06f);
        wait(1);
        go(0.3,0.3,-2.06f);
        rotate(0.30,-57);
    } else if(color1 == 1&&color == 0) {   //green
        wait(1);
    } else if(color1 == 1&&color == 1) {   //blue
        rotate(0.30,-60);
        wait(1);
        go(0.3,0.3,2.06f);
        wait(1);
        go(0.3,0.3,-2.06f);
        rotate(0.30,60);
    } else {
        wait(1);
    }

    go(0.6,0.6,7.05f);
    rotate(0.30,60);
    wait(1);
    go(0.6,0.6,5.37f);
    rotate(0.30,-60);
    wait(1);
    go(0.3,0.3,3.00f);
    rotate(0.30,-60);
    wait(1);

    openmv();                               //go to feed position
    rotate(0.50,-110);                       //turn 180
    wait(1);
    feed(0.25,0.25,2.00f);                  //feed!

    openmv1();                               //start backward process
    go(0.3,0.3,-1.50f);
    output = 1;
    wait(5);
    go(0.3,0.3,-6.00f);

    while(1);
}