/* Eff PID control Line Follwer Robot
        Team RUET Creakers */

#include<stdbool.h>

int trigpin = 15,echopin_r= 12,echopin_l=13,duration_l,distance_l,duration_r,distance_r,d_error;
int sensor[]={2,3,4,5,6,7,8,9};
boolean s_value[8],value[8];
int right_motor_forward=19,right_motor_backward=18,left_motor_forward=16,left_motor_backward=17;
int right_motor_pwm=10,left_motor_pwm=11;
//int serial_fq = 9600;
int sum_s_value,sensor_sum;
unsigned long m_time=0,c_time=0,d_cmtime=0;
float sensor_error=0,previous_error=0,position;
int ini_r_speed=100,ini_l_speed=100,del_speed;
int pro=0,inti=0,deri=0;
int left_motor_speed=0,right_motor_speed=0;
int last_position=104;
int start=0;

float kp=4.55,kd=1,ki=0;

void setup()
{
    //Serial.begin(serial_fq);

    for (int i = 2; i <=19; i++)
    {
        if (i==14)
        {
            continue;
        } 
        else if (i>=2 && i<=13)
        {
            if (i==10 || i==11)
            {
                pinMode(i,OUTPUT);
            }
            else
            {
                pinMode(i,INPUT);
            }         
        }
        else
        {
            pinMode(i,OUTPUT);
        }   
    }
}
void loop()
{
    sensor_read();
    c_time=millis();
    d_cmtime=c_time-m_time;
    if(last_position==100 || last_position==107)
    {
    if (d_cmtime>=500)
    {
    last_position=104;
    }
    }

if (sensor_sum==0)
{
    //Serial.print("  W\n");
    if (last_position==100)
    {   
        /*
        if (r_d==0)
        {
            analogWrite(right_motor_pwm,89);
            analogWrite(left_motor_pwm,80);
            sharp_right();
            delay(100);
            r_d=r_d+1;
        }
        */
        for (int i=0;s_value[3]==0 && s_value[4]==0;i++)
        {
            if(i==0)
            {
            analogWrite(right_motor_pwm,130);
            analogWrite(left_motor_pwm,108);
            sharp_right();
            delay(150);      
            }
            sensor_read();
            if (s_value[3]==1||s_value[4]==1)
            {
                last_position=104;
                break;
            }
            
            //Serial.print("  R\n"); 
            analogWrite(right_motor_pwm,100);
            analogWrite(left_motor_pwm,85);
            sharp_right();
        }
    }
    else if (last_position==107)
    {    
        /*    
        if (l_d==0)
        {
            analogWrite(right_motor_pwm,87);
            analogWrite(left_motor_pwm,83);
            sharp_left();
            delay(100);
            l_d=l_d+1; 
        }
        */
        for (int i=0;s_value[3]==0 && s_value[4]==0;i++)
        {
            if (i==0)
            {
            analogWrite(right_motor_pwm,85);
            analogWrite(left_motor_pwm,125);
            sharp_left();
            delay(150);    
            }
            
            sensor_read();
            if(s_value[3]==1 || s_value[4]==1)
            {
                last_position=104;
                break;
            }
            //Serial.print("  L\n");
            analogWrite(right_motor_pwm,78);
            analogWrite(left_motor_pwm,95);
            sharp_left();
        }          
    }
    else if (last_position==104)
    {
    for (;;)
    {
        sensor_read();
        if (sensor_sum>=1)
        {
            break;
        }
        right_motor_speed=60;
        left_motor_speed=50;
        forward();    
    }     
    }   
}

/*
else if (sensor_sum==3)
{
    if (s_value[0]==1 && s_value[1]==1 && s_value[2]==1 )
    {
            analogWrite(right_motor_pwm,60);
            analogWrite(left_motor_pwm,35);
            sharp_right();   
    }
    else if (s_value[5]==1 && s_value[6]==1 && s_value[7]==1)
    {
            Serial.print("  L\n");
            analogWrite(right_motor_pwm,85);
            analogWrite(left_motor_pwm,38);
            sharp_left();    
    }
    
}
*/

else if (sensor_sum==8)
{
    //Serial.print("  B\n");
    if(start==0)
    {
        start=1;
        for (;;)
        {
        sensor_read();
        right_motor_speed=55;
        left_motor_speed=60;
        forward();
        if (sensor_sum<5)
        {   
            last_position=104;
            break;
        }         
        }
    }
    else
    stop();
}
else
{
    pid();
    forward();
}

}
void sensor_read()
{
    for (int i = 0; i <=7; i++)
    {
      value[i]=digitalRead(sensor[i]);
      s_value[i] = !value[i] ;
      //Serial.print("  ");
      //Serial.print(i+1);
      //Serial.print(" ");
      //Serial.print(s_value[i]);
    }
    //Serial.print("");
    sum_s_value=s_value[0]*10+s_value[1]*20+s_value[2]*30+s_value[3]*40+s_value[4]*50+s_value[5]*60+s_value[6]*70+s_value[7]*80;
    sensor_sum=s_value[0]+s_value[1]+s_value[2]+s_value[3]+s_value[4]+s_value[5]+s_value[6]+s_value[7];
    position=sum_s_value/sensor_sum;
    sensor_error=position-45;
    
    if (s_value[0]==1 && s_value[1]==1)
    {
    last_position=100;
    m_time=millis();
    }
    else if (s_value[7]==1 && s_value[6]==1)
    {
    last_position=107;
    m_time=millis();
    }    
}
void pid()
{
    //Serial.print("    Error: ");
    //Serial.print(sensor_error);
    //Serial.print("    ");
    //Serial.print("Pre Error: ");
    //Serial.print(previous_error);
    pro=kp*sensor_error;
    deri=kd*(sensor_error-previous_error);
    inti=ki;
    del_speed=pro+deri+inti;
    del_speed=map(del_speed,-kp*35-10,kp*35+10,-65,65);
    //Serial.print("   Del speed: ");
    //Serial.print(del_speed);
    //Serial.print("  ");
    previous_error=sensor_error;
    left_motor_speed=ini_l_speed-del_speed;
    right_motor_speed=ini_r_speed+del_speed;
    //left_motor_speed=map(left_motor_speed,-70,290,20,100);
    //right_motor_speed=map(right_motor_speed,-70,290,20,100);
    //Serial.print("LS: ");
    //Serial.print(left_motor_speed);
    //Serial.print("  RS: ");
    //Serial.print(right_motor_speed);
    //Serial.print("\n");
}
void forward()
{
    analogWrite(right_motor_pwm,right_motor_speed);
    analogWrite(left_motor_pwm,left_motor_speed);
    digitalWrite(left_motor_forward,HIGH);
    digitalWrite(left_motor_backward,LOW);
    digitalWrite(right_motor_forward,HIGH);
    digitalWrite(right_motor_backward,LOW);
}
void backward()
{
    digitalWrite(left_motor_forward,LOW);
    digitalWrite(left_motor_backward,HIGH);
    digitalWrite(right_motor_forward,LOW);
    digitalWrite(right_motor_backward,HIGH);
}
void right()
{
    digitalWrite(left_motor_forward,HIGH);
    digitalWrite(left_motor_backward,LOW);
    digitalWrite(right_motor_forward,LOW);
    digitalWrite(right_motor_backward,LOW);    
}
void left()
{
    digitalWrite(left_motor_forward,LOW);
    digitalWrite(left_motor_backward,LOW);
    digitalWrite(right_motor_forward,HIGH);
    digitalWrite(right_motor_backward,LOW);     
}
void sharp_right()
{
    digitalWrite(left_motor_forward,HIGH);
    digitalWrite(left_motor_backward,LOW);
    digitalWrite(right_motor_forward,LOW);
    digitalWrite(right_motor_backward,HIGH);    
}
void sharp_left()
{
    digitalWrite(left_motor_forward,LOW);
    digitalWrite(left_motor_backward,HIGH);
    digitalWrite(right_motor_forward,HIGH);
    digitalWrite(right_motor_backward,LOW);     
}
void stop()
{
    analogWrite(right_motor_pwm,0);
    analogWrite(left_motor_pwm,0);
    digitalWrite(right_motor_forward,LOW);
    digitalWrite(right_motor_backward,LOW);
    digitalWrite(left_motor_forward,LOW);
    digitalWrite(left_motor_backward,LOW);
}