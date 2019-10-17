#include "mbed.h"
#include "Servo.h"
#include "math.h"
#include "DS3231.h"
Serial pc(PA_2, PA_3);
InterruptIn echo(PA_10);//PB_8---PB_2
DigitalOut trig(PB_14);
Servo myservo(PA_6);
DigitalOut output3(PA_4);//S0
DigitalOut output4(PA_5);//S1
DigitalOut output1(PA_8);
DigitalOut output2(PA_9);
InterruptIn input1(PB_15);//颜色传感器的输出PWM波的输入
DS3231 rtc(PB_11,PB_10);
void color_select();
void pluse1();
void pluse2();
void pluse3();

int m1,m2;
DigitalIn state(PB_12);
//DigitalIn state1(PB_10);
DigitalOut statereturn(PB_13);
DigitalOut statereturn1(PB_10);
DigitalOut led(PC_13);
int flag_color=0;
int step=0;
Timer A;
float dis=100;
int color=0;//1-r 2-g 3-b
int a=0,b=0,c=0;
int hour;
int minute;
int second;
int dayOfWeek;
int date;
int month;
int year; 
int signal= 1;

void start(void){
    A.start();
    }
    

void stop(void){
    A.stop();
    dis=A.read_us()/58.0;
    }

void measure(void){
    A.reset();
    trig=0;
    wait_ms(5);
    trig=1;
    wait_ms(15);
    trig=0;
    }
    
    
void ct()
{
    /*while(state.read()==0){
        wait_ms(1);
        }*/
    pc.printf("aaa--%3.2f",dis);
    wait_ms(10);
    while((dis>20)){                      //wait for ultrasonic start
       measure();
       pc.printf("%3.2f",dis);
       wait_ms(100);
        }                         //ultrasonic stop
    statereturn = 1;
    wait(2);                             
    for(int i =100;i>0;i--){                //feedfish start
        myservo = i/100.0;
        wait(0.1);
        }
    wait(2);
    myservo = 100/100.0;
    statereturn = 0;                             //feedfish stop
    dis = 100;
        
}



void color_read()
{
    a=0;
    b=0;
    c=0;//这三个量是用来数方波上升到高电平次数来确定频率的
    output3=1;
    output4=0;//这两个的设定没有影响
    //pc.printf("aaa");
        output1 = 0;
        output2 = 0;
        input1.rise(&pluse1);
        wait(0.02);//确定红入射光强度产生的频率
        output1 = 1;
        output2 = 1;
        input1.rise(&pluse2);
        wait(0.02);//确定绿入射光强度产生的频率
        output1 = 0;
        output2 = 1;
        input1.rise(&pluse3);
        wait(0.02);//确定蓝入射光强度产生的频率
        color_select();
        pc.printf("RED: %10d     GREEN: %10d     BLUE: %10d     ", a, b, c);
        wait(0.5);
        a=0;
        b=0;
        c=0;
        
}

void time()
{
        pc.printf("\r\n\r\n\r\n\r\n");
        //printf("\r\n\nDS3231 test program\r\n");
        //rtc.setI2Cfrequency(400000);
    
        rtc.writeRegister(DS3231_Aging_Offset,0); 
        //uncomment to set Aging Offset 1LSB = approx. 0.1 ppm according from datasheet = 0.05 ppm @ 21 °C from my measurments
     
        rtc.convertTemperature();
      
        int reg=rtc.readRegister(DS3231_Aging_Offset);
        if (reg>127)
            {reg=reg-256;}
        //pc.printf("Aging offset : %i\r\n",reg);
         
        //pc.printf("OSF flag : %i",rtc.OSF());
        //pc.printf("\r\n");
     
        rtc.readDate(&date,&month,&year);
        pc.printf("Date \t\t: %02i-%02i-%02i",date,month,year);
        pc.printf("\r\n");
     
        //rtc.setTime(10,17,00);
        // uncomment to set time//定义时间时分秒
     
        rtc.readTime(&hour,&minute,&second);
        pc.printf("Time \t\t: %02i:%02i:%02i",hour,minute,second);
        pc.printf("\r\n");
     
        //rtc.setDate(5,24,05,2019); 
        // uncomment to set date//在此刷新时间，天数
     
        rtc.readDateTime(&dayOfWeek,&date,&month,&year,&hour,&minute,&second);
        pc.printf("Date Time \t: %02i-%02i-%02i %02i:%02i:%02i",date,month,year,hour,minute,second);
        pc.printf("\r\n");
        
        //添加星期判断 
        //pc.printf("dayOfWeek: %i\r\n",dayOfWeek);
        //只显示数字
        
        switch(dayOfWeek)
        {
            case 1: pc.printf("Day \t\t: Monday ");break;
            case 2: pc.printf("Day \t\t: Tuesday ");break;
            case 3: pc.printf("Day \t\t: Wednesday ");break;
            case 4: pc.printf("Day \t\t: Thursday ");break;
            case 5: pc.printf("Day \t\t: Friday ");break;
            case 6: pc.printf("Day \t\t: Saturday ");break;
            case 7: pc.printf("Day \t\t: Sunday ");break;
        }
        pc.printf("\r\n");
        
        
        //pc.printf("Temperature \t:%6.2f",rtc.readTemp());
        //pc.printf("\r\n\r\n");
       
        
        pc.printf("team_number : 10\r\n");
        pc.printf("Team numbers:\r\n");
        pc.printf("\n-Si Hanche\t(2288915S)\r\n");
        pc.printf("\n-Lyh Qiyang\t(2288916L)\r\n");
        pc.printf("\n-Zhang Wenxiang\t(2288917Z)\r\n");
        pc.printf("\n-Jiang Xianhao\t(2288918J)\r\n");
        pc.printf("\n-Guan Haobin\t(2288919G)\r\n");
        pc.printf("\n-Lu liangtong\t(2288920L)\r\n");
        pc.printf("\n-Zhang Jiatai\t(2288921Z)\r\n");
        pc.printf("\n-Lai Shicen\t(2288922L)\r\n");
        
    
}


void color_select()
{
        if((a-3)>b){
           if(a>c){
               pc.printf("RED \r\n");
               statereturn=1;
               statereturn1=0;
               color=1;
               }
               else{
               pc.printf("BLUE \r\n");
               statereturn=1;
               statereturn1=1;
               color=3;
               }
        }
        else if(b>=c){
            pc.printf("GREEN \r\n");
            statereturn=0;
            statereturn1=1;
            color=2;
        }
        else{
            pc.printf("BLUE \r\n");
            statereturn=1;
            statereturn1=1;
            color=3;
        }
} //比较频率大小然后控制小灯闪烁

void pluse1(){
       a++;
       }

void pluse2(){
       b++;
       }
       
void pluse3(){
       c++;
       } 




int main() {
    state.mode(PullDown);
    statereturn=0;
    statereturn1=0;
    myservo=1;
    echo.rise(&start);
    echo.fall(&stop);
    //rtc.setTime(10,53,00);
    //rtc.setDate(5,24,05,2019); 
    while(1)
    {
        while(state.read()==1)
        {
            switch(step)
            {
                case 1:
                {
                    led=!led;
                    color_read();
                    wait(2);
                    statereturn=0;
                    statereturn1=0;
                    led=!led;
                    step++;
                    break;
                }
                case 2:
                {
                    led=!led;
                    ct();
                    led=!led;
                    step++;
                    break;
                }
                case 3:
                {
                    led=!led;
                    time();
                    led=!led;
                    step++;
                    break;
                }
            }
        }
        
    }
}
