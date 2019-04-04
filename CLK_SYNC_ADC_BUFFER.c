///////////////////////////////////////////////////////////////////////////////
///                     FPGA CLK SYCRONIZED ADC SAMPLER                     
///  READS ADC SUB 10X BOTH CHANNELS, SEPERATES 5 EA FOR READOUT AND EXP_OK
///  AVERAGES 5 SAMPS EA FOR REAOUT AND EXP_OK FOR BOTH CHANNELS
///  STORE THE AVERAGED SAMPLES IN RAM UNDER THE FOLLOWING VECTORS:             
///  VAL33_READOUT[10]                                                                                                                
///  VAL33_EXP_OK[10]                       
///  VAL35_READOUT[10]               
///  VAL35_EXP_OK[10]                                                                                                                 
///  THEN POPULATE FUNCTION CALL "GATHER_SUMMARY()" AFTER SAMPLES TAKEN   
///  MOVE EXT FROM 0 TO 2 (TP19)        
///  SAMPLES NOW 11 FROM 10                  
                                                                       
///////////////////////////////////////////////////////////////////////////////               
                             
#include <18F26K20.h>                      
#device ADC=8                                                    
                                                                                           
#FUSES NOBROWNOUT               //No brownout reset                     
#FUSES BORV27                   //Brownout reset at 2.7V                                                                                                                               
#FUSES NOMCLR                   //Master Clear pin used for I/O
#FUSES NOLVP                    //No low voltage prgming, B3(PIC16) or B5(PIC18) used for I/O
#FUSES NOXINST                  //Extended set extension and Indexed Addressing mode disabled (Legacy mode)                                                               
                                           
#use delay(internal=16000000)         
#use rs232(baud=115200,parity=N,xmit=PIN_C6,rcv=PIN_C7,bits=8,errors)                                                    
#use FIXED_IO( A_outputs=PIN_A7,PIN_A6,PIN_A5,PIN_A4 )                                                              
#use FIXED_IO( C_outputs=PIN_C2,PIN_C0 )                                                                                                                                        
#use pwm(CCP1,TIMER=2,FREQUENCY=1000,DUTY=0)                                      
                                                                                                                                             
#define P35V_RST      PIN_A4                                           
#define P33V_RST      PIN_A5                                              
#define RELAY1        PIN_A7                
#define RELAY2        PIN_A6                         
#define RELAY3        PIN_C0                            
#define ROWSHIFTIN    PIN_B0          
#define ROWCLK        PIN_B1                                
#define FPGA_SPARE0   PIN_B2         
#define FPGA_SPARE1   PIN_B3                                            
#define P35V_ALERT    PIN_B4                                                                      
#define P33V_ALERT    PIN_B5                                                                                                                                                              
#define INPUT1        PIN_C5   
#define REED_SWITCH   PIN_C1                                                  
#define PWM_LED       PIN_C2     
                                                                                                     
#define readout_ms   45   //PLANNED MODE IS 25FPS = 40ms PERIOD 22ms exposed, 18ms readout                                              
#define samples      11   //5 samps per exposed, 5 samps per readout                
                                                      
#include <ctype.h>                       
#include <float.h>           
#include <math.h>                                                                                                
#include <stddef.h>                               
#include <stdlib.h>                                
#include <stdlibm.h>                       
#include <string.h>                    
#include <stdio.h>
#include <stdint.h>                                                       
                                                                                                    
int32 A,TICK1=0;                                        
int16 B;                                                                   
unsigned int8 C;                                                                         
signed int8 D;                                                          
unsigned int8 x,v33currentold,v33currentbit,v35currentold,v35currentbit,buffer1,buffer2,buffer3,status1=0;      
INT8 loop=0;                              
int8 value_0[11],value_1[11],VAL33_READOUT[11],VAL33_EXP_OK[11],VAL35_READOUT[11],VAL35_EXP_OK[11]; 
int16 minv,maxv,i,y,Z,cnt,val,bufmax,value0_sum1,value0_sum2,value0_ave1,value0_ave2;    
int16 value1_sum1,value1_sum2,value1_ave1,value1_ave2; 
int relay_states=0;                                   
//int1 V33_P_flag=0,V33_N_flag=0,V35_P_flag=0,V35_N_flag=0;                                                   
int1 EXTflag=0,RDAflag=0,Pflag_Set=0;                                                          
                                                                     
void adc_sub();  
                      
void gather_summary();
                                                
                                                     
void main() {                                                 
                         
   pwm_set_duty_percent(0);                                                                                                                                                    
   pwm_on();                                   
   output_high(RELAY1);                                       
   output_low(RELAY2);                                  
   output_low(RELAY3);   
                                                          
   for(x=0;x<3;x++){                                     
      delay_ms(1000);                              
     printf("\n\CMOS BREAKOUT BOARD BOOTING UP IN %U\n\r",3-x);       
   }                                                 
                                                                          
                             
   enable_interrupts(INT_RDA);                                                                                                                    
   enable_interrupts(GLOBAL);                
                                                                                                                            
                                                                         
                                                                                                                                                              
   while (TRUE){                                 
                                          
      if(RDAflag==1){                                    
         disable_interrupts(INT_RDA);                  
         disable_interrupts(GLOBAL);                
                                                   
                                                                                                                                                                                                                    
         if (buffer1=='E'){// EXIT                                                
            delay_ms(50);                                                                             
            printf("E");                                                                                                                                   
         }                                                                    
         else if (buffer1=='L'){// LINK                                   
            delay_ms(50);                                        
            printf("L");                                         
         }                                     
         else if (buffer1=='X'){// LED PWM                                                               
            delay_ms(50);                                        
            printf("X");                                                                          
            printf("%U",buffer2);                                                                                     
            printf("%U",buffer3);                                                                                   
            //delay_ms(50);                           
            bufmax=buffer2;//INT8 into INT16 VAR      
            val= bufmax*3.92;                                                                                                          
            pwm_set_duty_percent(val);                                                                                                                                                    
            pwm_on();                               
            //}                                   
         }                                                                                  
         else if (buffer1=='R'){// SET RELAYS                                                                                                                                                                                                     
            delay_ms(50);                                                                 
            printf("R");                                       
            printf("%U",buffer2);                            
            printf("%U",buffer3);         

            relay_states=buffer2;                                         
                                             
            switch (buffer2) {                                                           
               case 0: output_low(RELAY1);  output_low(RELAY2),  output_low(RELAY3);  break; 
               case 1: output_high(RELAY1); output_low(RELAY2),  output_low(RELAY3);  break;                                         
               case 2: output_low(RELAY1);  output_high(RELAY2), output_low(RELAY3);  break; 
               case 3: output_high(RELAY1); output_high(RELAY2), output_low(RELAY3);  break;
               case 4: output_low(RELAY1);  output_low(RELAY2),  output_high(RELAY3); break; 
               case 5: output_high(RELAY1); output_low(RELAY2),  output_high(RELAY3); break;        
               case 6: output_low(RELAY1); output_high(RELAY2),  output_high(RELAY3); break;  
               case 7: output_high(RELAY1); output_high(RELAY2), output_high(RELAY3); break;                 
            } 

                                                    
         }                                                                  
         else if (buffer1=='K'){// STATUS BYTE                   
            delay_ms(50);                                          
            printf("K");                                                                         
            printf("%U",buffer2);                                                                
            printf("%U",buffer3);           
            printf("%U",0);             
            printf("%U",relay_states);//RELAYS 1 AND 2 ON                
                           
         }                                          
         else if (buffer1=='P'){// POWER SAMPLES    
            for(x=0;x<2;x++){
               output_high(RELAY3);
               delay_us(50);            
               output_low(RELAY3); 
               delay_us(50); 
            }                                           
            pflag_set=1;                                                
                                                                                                                   
         }                             
                                                         
         if (Pflag_Set==1){                                                  
                                                    
            ENABLE_INTERRUPTS(INT_EXT2_L2H); //uC CAN NOW DETECT USER SYNC 
            ENABLE_INTERRUPTS(GLOBAL);                                                                                                                                           
///////////////////////////////////////////////////////////////////////////////                                          
            do{ 
                                               
               loop++;//start a counter                    
                                              
               while(EXTflag==0);//WHILE FLAG IS 0 DO NOTHING 
               DISABLE_INTERRUPTS(INT_EXT2_L2H);//IMMEDIATELY CLEAR FLAG TO PREVENT RENTRANCE     
               DISABLE_INTERRUPTS(GLOBAL);               
                 
               if(EXTflag==1 && loop>1){                                                                    
                  adc_sub();                              
               }                                                                 
               EXTflag=0;                                                                                                      
               ENABLE_INTERRUPTS(INT_EXT2_L2H); //uC CAN NOW DETECT USER SYNC 
               ENABLE_INTERRUPTS(GLOBAL);   
                                               
            } while (loop<11);        
            
             loop=0;//RESET THE LOOP    
             DISABLE_INTERRUPTS(INT_EXT2_L2H);//IMMEDIATELY CLEAR FLAG TO PREVENT RENTRANCE  
             DISABLE_INTERRUPTS(GLOBAL);                                                                               
///////////////////////////////////////////////////////////////////////////////              

            printf("P ");                                      
            gather_summary();
            
         Pflag_Set=0;//ESCAPE LOOP  
         }                               
                                                                              
         RDAflag=0;                                  
         ENABLE_INTERRUPTS(INT_RDA); //THIS INT                              
         ENABLE_INTERRUPTS(GLOBAL);                        
      }  
      
      if(input(REED_SWITCH)){       
         delay_ms(75);//debounce                                                                         
         while(input(REED_SWITCH)){ 
            output_low(RELAY1);     
         }  
         output_high(RELAY1);
      } 
       
   }
}            
    
    
#INT_RDA                  //INT RDA SERIAL DATA HAS ARRIVED          
void  RDA_isr(void){                                                                                                                                      
   buffer1=getc(); //COMMAND                        
   buffer2=getc(); //B[1] //LIGHT DUTY CYCLE, RELAY STATES          
   buffer3=getc(); //B[2] //LIGHT ON OR OFF               
   RDAflag=1;                                           
}                          
                                                                     
#INT_EXT2                 //INTO EXTERNAL ISR - 1 FLAG IS SET HERE             
void  EXT2_isr(void){                                                                                                                                                                                                                                        
   EXTflag=1;                                                      
}        
    
   
void adc_sub(){                               
                                      
   setup_adc_ports(sAN0|sAN1, VSS_VREF);           
   setup_adc(ADC_CLOCK_DIV_16);            // Built-in A/D setup function                         
   //1us TAD so (4 + #ADCBITS)*TAD = 15us/sample   
                                                                                                                                                
    for(y=1;y<samples;y++){ // LOOPS THROUGH EACH OF THE 7 ANALOG CHANNELS                
                                                        
      output_high(RELAY3);               
                                       
      cnt=y;//SUMMING UP BOTH LOOPS AS A COUNTER FOR THE RAM POSITION        
                                    
      set_adc_channel(0);                                   
      delay_us(40);//SWITCH CHANNEL SETTING TIME                 
      value_0[cnt] = read_adc();                              
      set_adc_channel(1);                             
      delay_us(40);//SWITCH CHANNEL SETTLING TIME                                                   
      value_1[cnt] = read_adc();                                     
                                                      
      output_low(RELAY3);                                                                                     
                              
      //delay_ms(readout_ms/(samples-1));//DELAY BASED ON CMOS READOUT PERIOD/5 SAMPLES  
      delay_us(4750);//DELAY BASED ON CMOS READOUT PERIOD/5 SAMPLES
    }                                                                                                          
                                                         
///////////////////////////    3.3V CHANNEL    ////////////////////////////////      
////////////////////  SUM UP 5 3.3V MEASURMENTS FOR READ OUT  ///////////////// 
    value0_sum1=0;  
    for (x=1;x<6;x++){             
      value0_sum1 += value_0[x];        
    }                                                  
    value0_ave1 = value0_sum1/5;                     
                                                                                 
////////////////////  SUM UP 5 3.3V MEASURMENTS FOR EX OK  ////////////////////
    value0_sum2=0;  
    for (x=6;x<11;x++){                          
      value0_sum2 += value_0[x];               
    }                                 
    value0_ave2 = value0_sum2/5;                                                    
                                                                                                    
    VAL33_READOUT[loop] = value0_ave1;//<===================       
    VAL33_EXP_OK[loop]  = value0_ave2;//<===================       
                                                                                        
///////////////////////////    3.5V CHANNEL    ////////////////////////////////  
////////////////////  SUM UP 5 3.5V MEASURMENTS FOR READ OUT  ///////////////// 
    value1_sum1=0;       
    for (x=1;x<6;x++){
      value1_sum1 += value_1[x];               
    }                                           
    value1_ave1 = value1_sum1/5;             
                                               
////////////////////  SUM UP 5 3.5V MEASURMENTS FOR EX OK  ////////////////////     
    value1_sum2=0;  
    for (x=6;x<11;x++){  
      value1_sum2 += value_1[x];             
    }                                      
    value1_ave2 = value1_sum2/5;  
                          
    VAL35_READOUT[loop] = value1_ave1;//<===================    
    VAL35_EXP_OK[loop]  = value1_ave2;//<===================       
                                           
}                                                         
                                                                                                  
                                                                      
void gather_summary(){              
                                            
      for(x=2;x<12;x++){                       
         printf("%U ",VAL33_READOUT[x]);                
      }                                                      
      for(x=2;x<12;x++){                          
         printf("%U ",VAL33_EXP_OK[x]);                                      
      }                                                                                                           
      for(x=2;x<12;x++){                                                           
         printf("%U ",VAL35_READOUT[x]);                       
      }                                                                              
      for(x=2;x<12;x++){                          
         printf("%U ",VAL35_EXP_OK[x]);                              
      }                                                                                    
      printf("*\n\r");                                                      
                                                         
}                              
  
