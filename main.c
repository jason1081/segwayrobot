/* Name: main.c
 */

#include <avr/io.h>
#include "m_general.h"

#include "m_usb.h"
#include "m_imu.h"
#include "m_bus.h"

#define g 9.81
#define pi 3.14
#define freq_mimu 8000
#define alpha 0.02
#define accel_scale 2048.0
#define gyro_scale 16.4
#define freq_scale 400
#define Kp 175
#define Kd 450
#define Ki 200 //steady state error reduce
#define max_angle 0
#define min_angle -0
#define base_angle 0 //can be updated if values are different intially from the sensor

volatile uint8_t update_angle = 0;
volatile int flag_foward = 0, flag_back = 0;
volatile float angle;
volatile float output,factor;
volatile count = 0;
int data[9] = {0};
float Ax,Ay,Az,Gx,Gy,Gz,Mx,My,Mz;
float theta_Ay;
float angle_old = 0.0,delta_angle,integral_angle=0.0;
float timestep = 0.0025;

// to set the delta t for angle calculation
void Timer3_init(void){

    OCR3A = (unsigned int)62500/freq_scale; //Set OCR0A to get desired frequency (OCR1A = 62500/SamplingFrequency)
    //timer 3
    set(TCCR3B, CS32);		// Set Timer 3 Precaler to /256
    clear(TCCR3B, CS31);	//^
    clear(TCCR3B, CS30);		//^
 
    //timer 3 set the mode
    clear(TCCR3B,WGM33);// Set Timer 1 to count UP to OCR1A (mode 4), then reset
    set(TCCR3B,WGM32);
    clear(TCCR3A,WGM30);
     clear(TCCR3A,WGM31); 
    
    //timer 3 interuppt
    set(TIMSK3,OCIE3A);//Call Timer 1 interrupt whenever TCNT1 mathces OCR1A//TIMSK3 : OCIE3A
     
}

//to control the motor speed
void timer1_init(){//Initialize timer1
      //0 1 0 /8
    set(TCCR1B,CS10);// prescalar to 1//8
    //(mode 11) UP to OCR1A, DOWN to 0x0000, PWM mode
    //1 0 1 1
    set(TCCR1B,WGM13);
    set(TCCR1A,WGM11);
    set(TCCR1A,WGM10);
    //1 0
    //clear at OCR1B during UP, set at OCR1B during DOWN
    set(TCCR1A,COM1B1);
    set(TCCR1A,COM1C1);

}

void timer_update(){
    //int angle_shadow;
    OCR1A = 2000;
   factor = 0.09 * abs(output);//0.15//38.15
    if (factor > 1999){
        factor = 1999;
    }

    //controlling motor direction and accordingly setting OCR1 values
    if (flag_foward == 1){
        OCR1B = factor;//factor;//1000;//motor B
        flag_foward = 0;
    }
    if (flag_back == 1){
        OCR1C = factor;//factor;//1500;//motor C
        flag_back = 0;
    }
    // To set duty cycle,half of OCR1A to set duty cycle to 50%
    //m_clockdivide(0);//set system clock to 16MHz
    //set prescalar to 3,2MHz

}
int main(void)
{
    m_clockdivide(0);
    Timer3_init();//timer for setting delta t for angle value
     timer1_init();
    m_usb_init();
    m_imu_init(2, 3);
    sei();
    //for motor control-testing
    while(1)
    {
        if (update_angle) {
            
            m_imu_raw(data);
            Ax = (float)data[0]*g/accel_scale;
            Ay = (float)data[1]*g/accel_scale;
            Az = (float)data[2]*g/accel_scale;
            Gx = (float)data[3]/gyro_scale;
            Gy = (float)data[4]/gyro_scale;
            Gz = (float)data[5]/gyro_scale;
           // Mx = (float)data[6];
           // My = (float)data[7];
           // Mz = (float)data[8];
            
            
            //angle calculation
            theta_Ay = (float)Ay*90/g;
            angle = alpha*(angle_old + Gx*timestep*5)+ (1-alpha)*theta_Ay;
            angle_old = angle;
            delta_angle = angle_old + Gx*timestep*5;
            integral_angle += angle;
            //PID
            //output = (Kp*angle) + Kd*(angle - angle_old)*timestep + (Ki*integral_angle*timestep) ;
            output = Kp*angle + Kd*Gx*timestep + Ki*integral_angle*timestep;
            
            m_usb_tx_string("\t Ax: ");
            m_usb_tx_int(Ax);
            m_usb_tx_string("\t Ay: ");
            m_usb_tx_int(Ay);
            m_usb_tx_string("\t Az: ");
            m_usb_tx_int(Az);
            m_usb_tx_string("\t Gx: ");
            m_usb_tx_int(Gx);
            m_usb_tx_string("\t Gy: ");
            m_usb_tx_int(Gy);
            m_usb_tx_string("\t Gz: ");
            m_usb_tx_int(Gz);
            m_usb_tx_string("\t angle_Gx: ");
            m_usb_tx_int(angle);
            m_usb_tx_string("\t output: ");
            m_usb_tx_int(output);
     
            m_usb_tx_string("\n");
            update_angle = 0;
        }
        
        //PID Control
      // 
        
        //pwm , sensor , whell rotation spd, fbk loop, pwm generate;
        //pseudo code to control motor
        //      initialise timer for motor here, and set OCR1A,OCR1B,OCR1C value based on the speed u want to control the bot.
         //                  go slow or fast, angle less, slow, angle more faster
                
                
                if (angle > base_angle){//>
                    //move forward
                    set(DDRB,6);//set Port B pin 6 for output
                    clear(PORTB,6); //clear Port B pin 6}
                    clear(DDRB,7);
                    flag_foward = 1;
                    flag_back = 0;
                                    }
                if (angle < -base_angle){
                    //move backwards
                    set(DDRB,7);//set Port B pin 7 for output
                    clear(PORTB,7); //clear Port B pin 7
                    clear(DDRB,6);
                    flag_back = 1;
                    flag_foward = 0;
                }
        //timer1_init();//testing fr working
        timer_update();
    }
}

 ISR(TIMER3_COMPA_vect){
     update_angle = 1;

 }