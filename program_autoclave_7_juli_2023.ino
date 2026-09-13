


/** pin 
 * PC0 RELAY LAMPU 1  PDO RX                PB0 RELAY LAMPU 1       PA0 PS AIR           
 * PC1 RELAY LAMPU 2  PD1 TX                PB1 RELAY LAMPU 2       PA1 PS O2
 * PC2 RELAY ASTRAL   PD2                   PB2 RELAY astral        PA2 PS N2O2
 * PC3 RELAY INTRAO   PD3                   PB3 RELAY intrao        PA3 PS Ar
 * PC4 RELAY VIEWER   PD4 relay_telepon     PB4 RELAY viewer        PA4 PS N2
 * PC5 RELAY EXHAUST  PD5                   PB5 RELAY exhaust       PA5 PS Vacuum
 * PC6 RELAY MUTE     PD6                   PB6 MISO   PA6 
 * PC7                PD7                   PB7 SCK    PA7 
 * 
 * 
 * menu
 * 1 121 terbuka
 * 2 121 tertutup
 * 3 134 terbuka
 * 4 134 tertutup
 * 5 134 katun
 * 6 bd test
 * 7 test vacuum
 * 
 * proses
 * 1. he
 * 2. ur
 * 3. re
 * 4. dn
 * 5. st
 * 6. dry
 * 7. le
 * 8. ed
 * */
const int pA0 = 31; const int pB0=0; const int pC0=16; const int pD0=8;
const int pA1 = 30; const int pB1=1; const int pC1=17; const int pD1=9;
const int pA2 = 29; const int pB2=2; const int pC2=18; const int pD2=10;
const int pA3 = 28; const int pB3=3; const int pC3=19; const int pD3=11;
const int pA4 = 27; const int pB4=4; const int pC4=20; const int pD4=12;
const int pA5 = 26; const int pB5=5; const int pC5=21; const int pD5=13;
const int pA6 = 25; const int pB6=6; const int pC6=22; const int pD6=14;
const int pA7 = 24; const int pB7=7; const int pC7=23; const int pD7=15;

// ==================================== libery =======================================// 

// ============================================== rtc ==============================================//

#include <Wire.h>
#include <RTClib.h>
int detik, menit, jam, tanggal, bulan, tahun;
int data_waktu;
RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"Minggu", "Senin", "Selasa", "Rabu", "Kamis", "Jum'at", "Sabtu"};

// ==================================== eeprom =======================================// 

// call reset 
void(* resetFunc) (void) = 0;

// call reset end
#include <uEEPROMLib.h>
uEEPROMLib eeprom(0x57);

int var_suhu_ruang = 0,var_suhu_band = 0,var_suhu_heater = 0,var_pressure = 0;
int waktu_he = 10; // satuan menit
int waktu_ur = 8; // satuan menit
int waktu_re = 16; // satuan menit
int waktu_dn = 5; // satuan menit
int waktu_st_1 = 5; // 121 tertutup
int waktu_st_2 = 20; // 121 terbuka
int waktu_st_3 = 4; // 134 tertutup
int waktu_st_4 = 4; // 134 terbuka
int waktu_st_5 = 4; // 134 kain
int waktu_st_6 = 3; // bd test


int waktu_dry_1 = 15; // 121 tertutup
int waktu_dry_2 = 15; // 121 terbuka
int waktu_dry_3 = 4; // 134 tertutup
int waktu_dry_4 = 15; // 134 terbuka
int waktu_dry_5 = 20; // 134 kain
int waktu_dry_6 = 10; // bd test
int waktu_le = 10; // satuan menit
int waktu_tahun = 2023 ;
int waktu_bulan = 5 ;
int waktu_hari = 10 ;
int waktu_jam = 13 ;
int waktu_menit = 33 ;
int waktu_detik = 0 ;

// ==================================== sensor suhu band heater ==================================//
const int PT1000_PIN_heater2 = pA5;
const float vt_factor_heater2 = 1.88;
const float offset_heater2 = -30.50;
float temp_c_heater2;
float temp_sebelum_heater2,temp_sebelum2_heater2;
// ==================================== sensor suhu heater ==================================//
const int PT1000_PIN_heater1 = pA6;
const float vt_factor_heater1 = 1.88;
const float offset_heater1 = -22.50;
float temp_c_heater1;
float temp_sebelum_heater1,temp_sebelum2_heater1;
// ==================================== sensor suhu ruang ==================================//
const int PT1000_PIN = pA7;
const float vt_factor = 1.88;
const float vt_factor_tes = 8.4;

const float offset = -25.5;
float temp_c;
float temp_sebelum,temp_sebelum2;
int display_zero = 0;
// ==================================== pressure ===================================//
int rawValue; // A/D readings
int offset_press = 410; // zero pressure adjust
int fullScale = 9630; // max pressure (span) adjust
float pressure_value; // final pressure
// ==================================== led =========================================//
#include "LedControl.h"
LedControl lc=LedControl(5,7,6,2);
//LedControl lc2=LedControl(4,7,6,1);
byte nilai_lampu = B00000000;
byte nilai_lampu2 = B00000000;
int lampu_1,lampu_2,lampu_3,lampu_4,lampu_5,lampu_6,lampu_7,lampu_8,lampu_9,lampu_10,lampu_11;
// =================================== mux ==========================================//
int display_time=0;
#include <CD74HC4067.h>
// s0 s1 s2 s3
CD74HC4067 my_mux(pC2, pC3, pC4, pC5);  // create a new CD74HC4067 object with its four control pins
const int g_common_pin = pC6; // select a pin to share with the 16 channels of the CD74HC4067
//============================ variable ===============================================//
int point_menu = 1;
int point_cycle = 0;
int point_proses = 0;
int point_print = 0;
int point_error = 0;
int point_setting = 0;

float set_suhu_menu, set_pressure_menu;
float suhu_ruang, suhu_dinding,suhu_heater,pressure;
float control_suhu_heater, control_suhu_bandheater;

float waktu_st,waktu_drying;
bool flag_ganti_proses = false;
int jml_loop = 1;
int loop_run = 0;
int loop_tes_vacuum = 0;
int loop_stop = 0;
int menit_run = 0;
int nilai_vacuum = 0;
int jumlah_loop_panas = 0;
int loop_setting = 0;
int suhu_tertinggi = 0;
int suhu_terendah = 0;
int tekanan_tertinggi = 0;
int tekanan_terendah = 0;


bool flag_start = false;
bool flag_tes_vacuum = false;
bool flag_print = false;

bool flag_heater = false;
bool flag_bandheater = false;
bool flag_vacuum = false;
bool flag_pompa = false;
bool flag_fan = false;
bool flag_error_heater = false;
bool flag_cycle = false;
bool flag_Siklus= true;
bool flag_hasil_vacum = false;
bool flag_alarm_tangki = false;
bool flag_alarm_buzzer = false;
bool flag_jeda_buzzer = false;
bool flagSetRTC = false;
bool hold_add_rtc = false;
bool change_add_rtc = false;
bool flag_selesai_print = false;

bool flag_ledbersih_alarm = false;
bool flag_jeda_alarm_ledbersih = false;
bool flag_ledkotor_alarm = false;
bool flag_jeda_alarm_ledkotor = false;
bool flag_setting = false;
bool flag_setting_jam = false;
bool flag_delay_setting = false;

unsigned long millis_vacumtest;
const unsigned long millis_flagvacumtest = 1000;
unsigned long millis_stop;
const unsigned long millis_flagstop = 2000;
unsigned long millis_menit;
const unsigned long millis_flagsmenit = 60000;
unsigned long millis_pompa;
const unsigned long millis_flagspompa = 10000;
unsigned long millis_high_temp;
const unsigned long millis_flagshigh_temp = 1000;
unsigned long millis_baca_sensor;
const unsigned long millis_flagsbacasensor = 3000;
unsigned long millis_jeda;
const unsigned long millis_flagsjeda = 10000;
unsigned long millis_panas;
const unsigned long millis_flagspanas = 20000;
unsigned long millis_cycle;
const unsigned long millis_flagcycle = 2000;

unsigned long millis_display_suhu;
const unsigned long millis_flagdisplay_suhu = 1000;

unsigned long millis_alarm_tangki;
const unsigned long millis_flagalarm_tangki = 2000;

unsigned long millis_error;
unsigned long millis_flagserror = 1000;
unsigned long millis_pintu;
const unsigned long millis_flagspintu = 3000;

unsigned long millis_buzzer;
const unsigned long millis_flagsbuzzer = 1000;
unsigned long millis_ledbersih_alarm;
const unsigned long millis_flagsledbersih_alarm = 1000;
unsigned long millis_ledkotor_alarm;
const unsigned long millis_flagsledkotor_alarm = 1000;


unsigned long millis_setting;
const unsigned long millis_flagsetting = 20000;


unsigned long millis_serialprint;
const unsigned long millis_flagsserialprint = 60000;

unsigned long millis_alarm;
const unsigned long millis_flagsalarm = 4000;

void pembacaan_suhu_heater()
{
    float selisih,selisih2;
    int sensorvalue = analogRead(PT1000_PIN_heater2);
    float voltage = sensorvalue * (5.0 / 1023.0);
    temp_c_heater2 = (((voltage * 100) / vt_factor_heater2) + offset_heater2);
    if(temp_c_heater2 < 5.00)
    {
      point_error = 1;
      point_menu = 8;
    }
    
    if(!temp_sebelum_heater2)
    {
      temp_sebelum_heater2 = temp_c_heater2;
      selisih = temp_c_heater2;
    }else{
      selisih = (temp_sebelum_heater2 + temp_c_heater2)/2;
      temp_sebelum_heater2 = selisih;
      if(!temp_sebelum2_heater2)
      {
        temp_sebelum2_heater2 = temp_sebelum_heater2;
        selisih2 = temp_sebelum_heater2;
      }else{
        selisih2 = (temp_sebelum_heater2 + temp_sebelum2_heater2)/2;
        temp_sebelum2_heater2 = selisih2;
        
      }
    }
    temp_c_heater2 = selisih2;
   // Serial.print("nilai : ");
   // Serial.print(sensorvalue);
    
   // Serial.print("heater : ");
   // Serial.println(selisih2);
}

void pembacaan_suhu_bandheater()
{   float selisih,selisih2;
    int sensorvalue = analogRead(PT1000_PIN_heater1);
    float voltage = sensorvalue * (5.0 / 1023.0);
    temp_c_heater1 = (((voltage * 100) / vt_factor_heater1) + offset_heater1);
    if(!temp_sebelum_heater1)
    {
      temp_sebelum_heater1 = temp_c_heater1;
      selisih = temp_c_heater1;
    }else{
      selisih = (temp_sebelum_heater1 + temp_c_heater1)/2;
      temp_sebelum_heater1 = selisih;
      if(!temp_sebelum2_heater1)
      {
        temp_sebelum2_heater1 = temp_sebelum_heater1;
        selisih2 = temp_sebelum_heater1;
      }else{
        selisih2 = (temp_sebelum_heater1 + temp_sebelum2_heater1)/2;
        temp_sebelum2_heater1 = selisih2;
        
      }
    }
   
    if(temp_c_heater1 < 3.00)
    {
      point_error = 1;
      point_menu = 8;
    }
    temp_c_heater1 = selisih2;
    //Serial.print("nilai : ");
    //Serial.print(sensorvalue);
    //Serial.print(" band : ");
    //Serial.println(selisih2);
    
    
}

void pembacaan_suhu_ruang()
{
    float selisih,selisih2;
    int sensorvalue = analogRead(PT1000_PIN);
    float voltage = sensorvalue * (5.00 / 1023.0);
    float nilai_set = sensorvalue / vt_factor_tes;
    temp_c = (((voltage * 100) / vt_factor) + offset);
    if(!temp_sebelum)
    {
      temp_sebelum = temp_c;
      selisih = temp_c;
    }else{
      selisih = (temp_sebelum + temp_c)/2;
      temp_sebelum = selisih;
      if(!temp_sebelum2)
      {
        temp_sebelum2 = temp_sebelum;
        selisih2 = temp_sebelum;
      }else{
        selisih2 = (temp_sebelum + temp_sebelum2)/2;
        temp_sebelum2 = selisih2;
        
      }
    }
    float set_sel = selisih2;
    float fix = set_sel + (set_sel*((set_sel-22.00)*2.8/100)); 
    
    if(temp_c < 3.00)
    {
      point_error = 1;
      point_menu = 8;
    }else if(temp_c > 140.00)
    {
      point_error = 3;
      point_menu = 8;
    }
    
    
    temp_c = temp_sebelum2;
    if( temp_c > 40.0 )
    {
      temp_c = temp_c + 2.0;
    }else if(temp_c > 50.0)
    {
      temp_c = temp_c + 4.0;
    }else if(temp_c > 60.0)
    {
      temp_c = temp_c + 5.0;
    }else if(temp_c > 70.0)
    {
      temp_c = temp_c + 6.0;
    }else if(temp_c > 80.0)
    {
      temp_c = temp_c + 8.0;
    }else if(temp_c > 90.0)
    {
      temp_c = temp_c + 9.0;
    }else if(temp_c > 100.0)
    {
      temp_c = temp_c + 10.0;
    }else if(temp_c > 105.0)
    {
      temp_c = temp_c + 13.0;
    }else if(temp_c > 115.0)
    {
      temp_c = temp_c + 14.0;
    }else{
      
    }
    //Serial.print("pressure :");
    //Serial.println(pressure_value);
    if(flag_setting == true)
    {
      
    }else{
      int nilai = temp_c*10;
      int nilai_koma = (nilai)%10;
      int nilai_satuan = (nilai/10 )%10;
      int nilai_puluhan = (nilai /100)%10;
      int nilai_ratusan = (nilai / 1000)%10;
      int s = digitalRead(pC6);
      if(point_menu == 7 && point_proses == 8)
       {
  
       }else{
        if(flag_start == true)
        {
          lc.setDigit(0,6,nilai_ratusan,false);
          lc.setDigit(0,3,nilai_puluhan,false);
          // lc.setDigit(0,5,nilai_satuan,true);
          lc.setDigit(0,4,nilai_satuan,true);
          // lc.setDigit(0,6,nilai_koma,false);
          lc.setDigit(0,5,nilai_koma,false);   
        }else{
          
            if(display_time == 0)
            {
              lc.setRow(0,6,B00000001);
              lc.setRow(0,3,B00000000);
              lc.setRow(0,4,B00000000);
              lc.setRow(0,5,B00000000);
              display_time=1;
            }else if(display_time == 1)
            {
              lc.setRow(0,6,B00000000);
              lc.setRow(0,3,B00000001);
              lc.setRow(0,4,B00000000);
              lc.setRow(0,5,B00000000);
              display_time=2;
            }else if(display_time == 2)
            {
              lc.setRow(0,6,B00000000);
              lc.setRow(0,3,B00000000);
              lc.setRow(0,4,B00000001);
              lc.setRow(0,5,B00000000);
              display_time=3;
            }else if(display_time == 3)
            {
              lc.setRow(0,6,B00000000);
              lc.setRow(0,3,B00000000);
              lc.setRow(0,4,B00000000);
              lc.setRow(0,5,B00000001);
              display_time=0;
            }

          
          
          
        }
       
       }
    }
    
    
    
    //Serial.print("nilai : ");
    //Serial.print(sensorvalue);
    //Serial.print(" suhu_ruang : ");
    //Serial.println(temp_c);
}

void pembacaan_pressure()
{
    rawValue = 0;
    for (int x = 0; x < 10; x++) rawValue = rawValue + analogRead(pA4);
    pressure_value = (rawValue - offset_press) * 700.0 / (fullScale - offset_press) - 104.1; // pressure conversion
    

     if(point_menu == 7)
      {
        if(pressure_value < 20 )
        {
          pressure_value = pressure_value - 1.0;
        }else if(pressure_value < 30 )
        {
            pressure_value = pressure_value - 2.0;
          
        }else if(pressure_value < 40 )
        {
            pressure_value = pressure_value - 3.0;
          
        }else if(pressure_value < 45 )
        {
            pressure_value = pressure_value - 4.0;
          
        }else if(pressure_value < 50 )
        {
            pressure_value = pressure_value - 5.0;
          
        }else{
          
        }
      }else{
        /*if(pressure_value < 5 )
        {
          pressure_value = pressure_value - 1.0;
        }else if(pressure_value < 10 )
        {
            pressure_value = pressure_value - 3.0;
          
        }else if(pressure_value < 15 )
        {
            pressure_value = pressure_value - 5.0;
          
        }else if(pressure_value < 20 )
        {
            pressure_value = pressure_value - 7.0;
          
        }else if(pressure_value < 25 )
        {
            pressure_value = pressure_value - 9.0;
          
        }else if(pressure_value < 30 )
        {
            pressure_value = pressure_value - 11.0;
          
        }else if(pressure_value < 35 )
        {
            pressure_value = pressure_value - 13.0;
          
        }else if(pressure_value < 40 )
        {
            pressure_value = pressure_value - 15.0;
          
        }else if(pressure_value < 45 )
        {
            pressure_value = pressure_value - 17.0;
          
        }else if(pressure_value < 50 )
        {
            pressure_value = pressure_value - 19.0;
          
        }else if(pressure_value < 55 )
        {
            pressure_value = pressure_value - 21.0;
          
        }else if(pressure_value < 60 )
        {
            pressure_value = pressure_value - 23.0;
          
        }else{
          
        } */
    }

    
    int pressure_nilai = 0;
    int nilai_satuan = 0;
    int nilai_puluhan = 0;
    int nilai_ratusan = 0;
    //Serial.print("nilai : ");
    //Serial.print(rawValue);
    //Serial.print(" pressure : ");
    //Serial.println(pressure_value);
    if(flag_setting == true)
    {
     
    }else{
      if(pressure_value <0)
      {
         pressure_nilai = abs(pressure_value);
        
        
         nilai_satuan = (pressure_nilai)%10;
         nilai_puluhan = (pressure_nilai /10)%10;
         nilai_ratusan = (pressure_nilai / 100)%10;
         //Serial.print("-");
          lc.setRow(0,0,B00000001);
          lc.setDigit(0,1,nilai_puluhan,false);
          lc.setDigit(0,2,nilai_satuan,false);
          //lc.setRow(0,3,B00000000);
      }else{
         pressure_nilai = pressure_value;
        
        
         nilai_satuan = (pressure_nilai)%10;
         nilai_puluhan = (pressure_nilai /10)%10;
         nilai_ratusan = (pressure_nilai / 100)%10;
  
         lc.setDigit(0,0,nilai_ratusan,false);
          lc.setDigit(0,1,nilai_puluhan,false);
          lc.setDigit(0,2,nilai_satuan,false);
      }  
    }
    
    
    
    
}
void proses()
{
  int nilai_pintu = digitalRead(pA3);
    DateTime now = rtc.now();
  switch (point_proses) {
    case 1: // 1. he
      flag_selesai_print = true;
     // Serial.println(nilai_pintu);
      digitalWrite(pB0,LOW);
      digitalWrite(pB1,LOW);
      digitalWrite(pB2,LOW);
      digitalWrite(pB3,LOW);
      digitalWrite(pB4,LOW);
      lc.setRow(1,0,B00110111);
      lc.setRow(1,1,B01001111);
      if(flag_error_heater == false )
      {
        millis_error = millis();
        millis_flagserror = 360000;
        flag_error_heater = true;
      }
      if(nilai_pintu == LOW)
      {
        point_error = 6;
        point_menu = 8;
        flag_error_heater = false;
        point_proses = 8;
        loop_run =0;
        flag_start = false;
        millis_error = millis();
        
      }
      if(millis()- millis_error > millis_flagserror){
        
        point_error = 4;
        point_menu = 8;
        flag_error_heater = false;
        point_proses = 0;
        loop_run =0;
        flag_start = false;
        millis_error = millis();
        
       }
      warmup_he();
    break;
    case 2: // 2. ur
      lc.setRow(1,0,B00111110);
      lc.setRow(1,1,B00000101);
      digitalWrite(pB0,HIGH);
      digitalWrite(pB1,HIGH);
      digitalWrite(pB2,LOW);
      digitalWrite(pB3,LOW);
      digitalWrite(pB4,LOW);
      //millis_error = millis();
      if(flag_ganti_proses == false )
      {
        loop_run = loop_run  + 1;
        flag_ganti_proses = true;
      }
      
     // point_proses = 3 ;
       if(pressure_value <0)
       {
         int nilai = abs(pressure_value);
         if(nilai > 81)
        {
          Serial.print(now.hour(), DEC); //jam
          Serial.print("."); 
          Serial.print(now.minute(), DEC); //menit
          Serial.print("  "); 
          Serial.print(temp_c);
          Serial.print("C   "); // SUHU
          Serial.print(pressure_value);
          Serial.print("KPa  "); //TEKANAN
          Serial.println("Ur"); // MODE Vacuum
          
          millis_flagserror = 960000;
          millis_error = millis();
          point_proses = 3 ;
        }
       }
       if(millis()- millis_error > millis_flagserror){
        point_menu = 8;
        point_error = 2;
        point_proses = 0;
        loop_run =0;
        flag_start = false;
       }
       if(nilai_pintu == LOW)
      {
        point_error = 6;
        point_menu = 8;
        flag_error_heater = false;
        point_proses = 8;
        loop_run =0;
        flag_start = false;
        millis_error = millis();
        
      }
      //point_proses = 3 ;
    break;
    case 3: // 3. Re
      //Serial.println("3.rE");
      pompa();
      flag_ganti_proses = false;
      lc.setRow(1,0,B00000101);
      lc.setRow(1,1,B01001111);
      digitalWrite(pB0,LOW); // s1
      digitalWrite(pB1,LOW); // sb
      digitalWrite(pB3,HIGH); // scv
      if(loop_run >= 2 && point_menu == 1 || loop_run >= 2 && point_menu == 3 || loop_run >= 3 && point_menu == 2 || loop_run >= 3 && point_menu == 4 || loop_run >= 3 && point_menu == 5 || loop_run >= 3 && point_menu == 6)
      {
        
        int nilai = pressure_value;
        suhu_terendah = 140;
        tekanan_terendah = 230;
        if(nilai > set_pressure_menu )
         {
          Serial.print(now.hour(), DEC); //jam
          Serial.print("."); 
          Serial.print(now.minute(), DEC); //menit
          Serial.print("  "); 
          Serial.print(temp_c);
          Serial.print("C   "); // SUHU
          Serial.print(pressure_value);
          Serial.print("KPa  "); //TEKANAN
          if(point_proses == 3 )
          {
            Serial.println("Re"); // MODE Penambahan Tekanan
          }else if(point_proses == 11 )
          {
            Serial.println("Err"); // MODE Error
            Serial.println("Proses error");
          }
          millis_menit = millis();
          millis_high_temp = millis();
          menit_run = waktu_st;
          millis_jeda = millis();
           point_proses = 5 ;  
         }
      }else{
        int nilai = pressure_value;
        if(nilai >= 50 )
         {
          Serial.print(now.hour(), DEC); //jam
          Serial.print("."); 
          Serial.print(now.minute(), DEC); //menit
          Serial.print("  "); 
          Serial.print(temp_c);
          Serial.print("C   "); // SUHU
          Serial.print(pressure_value);
          Serial.print("KPa  "); //TEKANAN
          if(point_proses == 3 )
          {
            Serial.println("Re"); // MODE Penambahan Tekanan
          }else if(point_proses == 11 )
          {
            Serial.println("Err"); // MODE Error
            Serial.println("Proses error");
          }
          millis_flagserror = 300000;
          millis_error = millis();
           point_proses = 4 ;  
         }
      }
      if(millis()- millis_error > millis_flagserror){
        point_menu = 8;
        point_error = 8;
        //point_proses = 0;
        loop_run =0;
        flag_start = false;
       }
       if(nilai_pintu == LOW)
      {
        point_error = 6;
        point_menu = 8;
        flag_error_heater = false;
        point_proses = 8;
        loop_run =0;
        flag_start = false;
        millis_error = millis();
        
      }
    break;
    case 4: // 4. dn
      //Serial.println("4.dn");
      lc.setRow(1,0,B00111101);
      lc.setRow(1,1,B00010101);
      digitalWrite(pD2,HIGH);
      digitalWrite(pB0,HIGH);
      digitalWrite(pB1,LOW);
      digitalWrite(pB2,HIGH);
      digitalWrite(pB3,HIGH);
      if( pressure_value < 15 )
      {
         //loop_run = loop_run  + 1;
        if(point_menu == 1)
        {
          Serial.print(now.hour(), DEC); //jam
          Serial.print("."); 
          Serial.print(now.minute(), DEC); //menit
          Serial.print("  "); 
          Serial.print(temp_c);
          Serial.print("C   "); // SUHU
          Serial.print(pressure_value);
          Serial.print("KPa  "); //TEKANAN
            Serial.println("Dn"); // MODE DN ke Vacuum
          
          loop_run = loop_run  + 1;
          millis_flagserror = 1200000;
          millis_error = millis();
          point_proses = 3 ;
        } else{
          Serial.print(now.hour(), DEC); //jam
          Serial.print("."); 
          Serial.print(now.minute(), DEC); //menit
          Serial.print("  "); 
          Serial.print(temp_c);
          Serial.print("C   "); // SUHU
          Serial.print(pressure_value);
          Serial.print("KPa  "); //TEKANAN
          Serial.println("Dn"); // MODE DN ke Vacuum
          
         // loop_run = loop_run  + 1;
          millis_flagserror = 1200000;
          millis_error = millis();
          point_proses = 2 ;
        }
        
        
      }
      if(millis()- millis_error > millis_flagserror){
        point_menu = 8;
        point_error = 5;
        //point_proses = 0;
        loop_run =0;
        flag_start = false;
       }
       if(nilai_pintu == LOW)
      {
        point_error = 6;
        point_menu = 8;
        flag_error_heater = false;
        point_proses = 8;
        loop_run =0;
        flag_start = false;
        millis_error = millis();
        
      }
    break;
    case 5: // 5. st
      digitalWrite(pB1,LOW);
      digitalWrite(pB3,HIGH);
      
      if(millis()- millis_menit > millis_flagsmenit){
        menit_run = menit_run - 1;
        if(suhu_tertinggi < temp_c)
        {
          suhu_tertinggi = temp_c;
        }
  
        if(suhu_terendah > temp_c)
        {
          suhu_terendah = temp_c;
        }
        
        if(tekanan_tertinggi < pressure_value)
        {
          tekanan_tertinggi = pressure_value;
        }
  
        if(tekanan_terendah > pressure_value)
        {
          tekanan_terendah = pressure_value;
        }
        
        millis_menit = millis();
      }
      if(millis()- millis_jeda > millis_flagsjeda){
        int time_st = menit_run%10;
         lc.setDigit(1,1,time_st,false);
         time_st = (menit_run/10)%10;
         lc.setDigit(1,0,time_st,false);
         
       }else{
        lc.setRow(1,0,B00111110);
        lc.setRow(1,1,B01110111);
       }
      //int nilai_pressure = pressure_value;
      if( pressure_value < set_pressure_menu)
      {
        pompa();
      }else{
        digitalWrite(pD2,HIGH);
        digitalWrite(pB2,LOW);
      }

      if(temp_c > (set_suhu_menu + 2.50))
      {
        if(millis()- millis_high_temp > millis_flagshigh_temp ){
          digitalWrite(pB1,HIGH);
          digitalWrite(pB3,LOW);
          delay(500);
          digitalWrite(pB1,LOW);
          digitalWrite(pB3,HIGH);
          millis_high_temp=millis();
        }
        
      }
      
      if(menit_run == 0)
      {
        Serial.print(now.hour(), DEC); //jam
          Serial.print("."); 
          Serial.print(now.minute(), DEC); //menit
          Serial.print("max  "); 
          Serial.print(suhu_tertinggi);
          Serial.print("C   "); // SUHU tertinggi
          Serial.print("min  "); 
          Serial.print(suhu_terendah);
          Serial.print("C   "); // SUHU terendah
          
          Serial.print("max  "); 
          Serial.print(tekanan_tertinggi);
          Serial.print("KPa  "); //TEKANAN tertinggi
          Serial.print("min  "); 
          Serial.print(tekanan_terendah);
          Serial.print("KPa  "); //TEKANAN terendah
          
          Serial.println("St"); // MODE Sterilisasi
          
         millis_flagserror = 50000;
         millis_error = millis();
        point_proses = 9 ;  
      }
      if(nilai_pintu == LOW)
      {
        point_error = 6;
        point_menu = 8;
        flag_error_heater = false;
        point_proses = 8;
        loop_run =0;
        flag_start = false;
        millis_error = millis();
        
      }
    break;
    case 6: // 6. dry
      //Serial.println("6.dry");
       digitalWrite(pB0,HIGH);
      digitalWrite(pB1,HIGH);
      digitalWrite(pB2,LOW);
      digitalWrite(pB3,LOW);
      digitalWrite(pB4,LOW);
      if(millis()- millis_menit > millis_flagsmenit){
        menit_run = menit_run - 1;
        millis_menit = millis();
      }
      if(millis()- millis_jeda > millis_flagsjeda){
        int time_st = menit_run%10;
         lc.setDigit(1,1,time_st,false);
         time_st = (menit_run/10)%10;
         lc.setDigit(1,0,time_st,false);
         
       }else{
        lc.setRow(1,0,B00111101);
        lc.setRow(1,1,B00010101);
       }
       
      //Serial.print("menit ");
      //Serial.println(menit_run);
      if(menit_run == 1)
      {
        digitalWrite(pB4,HIGH); 
      }
      if(menit_run == 2 )
         {
          flag_hasil_vacum == false;
          if(pressure_value <0)
          {
            int nilai = abs(pressure_value);
            if(nilai > 76)
            {
              nilai_vacuum = 1;
            }else{
              nilai_vacuum = 2;
            }
          }
         }
      if(menit_run == 0)
      {
          Serial.print(now.hour(), DEC); //jam
          Serial.print("."); 
          Serial.print(now.minute(), DEC); //menit
          Serial.print("  "); 
          Serial.print(temp_c);
          Serial.print("C   "); // SUHU
          Serial.print(pressure_value);
          Serial.print("KPa  "); //TEKANAN
          Serial.println("Dry"); // MODE DRY
          
        point_proses = 7 ;  
      }
      if(nilai_pintu == LOW)
      {
        point_error = 6;
        point_menu = 8;
        flag_error_heater = false;
        point_proses = 8;
        loop_run =0;
        flag_start = false;
        millis_error = millis();
        
      }
      //point_proses = 7 ;
    break;
    case 7: // 7. le
      //Serial.println("7.le");
      lc.setRow(1,0,B00001110);
      lc.setRow(1,1,B01001111);
      digitalWrite(pB4,HIGH);
      if(pressure_value <0)
       {
         int nilai = abs(pressure_value);
         if(nilai < 1 )
        {
          
         Serial.print(now.hour(), DEC); //jam
          Serial.print("."); 
          Serial.print(now.minute(), DEC); //menit
          Serial.print("  "); 
          Serial.print(temp_c);
          Serial.print("C   "); // SUHU
          Serial.print(pressure_value);
          Serial.print("KPa  "); //TEKANAN
          Serial.println("Le"); // MODE Melepas Tekanan ke proses selesai
          
        point_proses = 8 ;
        Serial.print(now.hour(), DEC); //jam
          Serial.print("."); 
          Serial.print(now.minute(), DEC); //menit
          Serial.print("  "); 
          Serial.print(temp_c);
          Serial.print("C   "); // SUHU
          Serial.print(pressure_value);
          Serial.print("KPa  "); //TEKANAN
          
            Serial.println("ED"); // MODE Melepas Tekanan ke proses selesai
          
          
          
         // Serial.println("2.Ur");
        }
       }else if (pressure_value > 0){
        Serial.print(now.hour(), DEC); //jam
          Serial.print("."); 
          Serial.print(now.minute(), DEC); //menit
          Serial.print("  "); 
          Serial.print(temp_c);
          Serial.print("C   "); // SUHU
          Serial.print(pressure_value);
          Serial.print("KPa  "); //TEKANAN
          Serial.println("Le"); // MODE Melepas Tekanan ke proses selesai
          
        point_proses = 8 ;
        Serial.print(now.hour(), DEC); //jam
          Serial.print("."); 
          Serial.print(now.minute(), DEC); //menit
          Serial.print("  "); 
          Serial.print(temp_c);
          Serial.print("C   "); // SUHU
          Serial.print(pressure_value);
          Serial.print("KPa  "); //TEKANAN
          
            Serial.println("ED"); // MODE Melepas Tekanan ke proses selesai
          
       }
       if(nilai_pintu == LOW)
      {
        //Serial.println("pinttuuuuuu bukaaaa------------------------------------");
        point_error = 6;
        point_menu = 8;
        flag_error_heater = false;
        point_proses = 8;
        loop_run =0;
        flag_start = false;
        millis_error = millis();
        
      }
      //point_proses = 8 ;
    break;
    case 8: // 8. ed
      //Serial.println("8.ed");
      lc.setRow(1,0,B01001111);
      lc.setRow(1,1,B00111101);
      digitalWrite(pB0,LOW); 
      digitalWrite(pB1,LOW); 
      digitalWrite(pB2,LOW); 
      digitalWrite(pB3,LOW); 
      digitalWrite(pB4,LOW);
      loop_run =0; 
      if(point_menu == 7 && nilai_vacuum == 1 )
      {
        lc.setRow(0,6,B01111011);
        lc.setRow(0,3,B01111110);
        lc.setRow(0,4,B01111110);
        lc.setRow(0,5,B00111101);
        if(flag_hasil_vacum == false)
        {
          Serial.println("");
          Serial.println("Vacuum Test : PASS"); // MODE Selesai
          flag_hasil_vacum = true;  
        }
        
       // flag_print = false ;  
      }else if(point_menu == 7 && nilai_vacuum == 2 && flag_hasil_vacum == false){
        lc.setRow(0,6,B00000000);
        lc.setRow(0,3,B00011111);
        lc.setRow(0,4,B01110111);
        lc.setRow(0,5,B00111101);
        if(flag_hasil_vacum == false)
        {
          Serial.println("");
          Serial.println("Vacuum Test : NOT PASS"); // MODE Selesai
          flag_hasil_vacum = true;  
        }
        
       // flag_print = false ;
      }

       if(nilai_pintu == LOW)
      {
        //point_error = 6;
        //point_menu = 8;
        //flag_error_heater = false;
        point_proses = 0;
        loop_run =0;
        flag_start = false;
        millis_error = millis();
        
      }
    break;
    case 9: // 4. dn setelah st
      //Serial.println("4.dn");
      lc.setRow(1,0,B00111101);
      lc.setRow(1,1,B00010101);
      
      digitalWrite(pB0,HIGH);
      digitalWrite(pB1,LOW);
      digitalWrite(pB2,HIGH);
      digitalWrite(pB3,HIGH);
      if( pressure_value < 15 )
      {
        
            Serial.print(now.hour(), DEC); //jam
          Serial.print("."); 
          Serial.print(now.minute(), DEC); //menit
          Serial.print("  "); 
          Serial.print(temp_c);
          Serial.print("C   "); // SUHU
          Serial.print(pressure_value);
          Serial.print("KPa  "); //TEKANAN
            Serial.println("Dn"); // MODE DN ke Vacuum
          
        //point_proses = 6 ;
            menit_run = waktu_drying ;
            millis_menit = millis();
            point_proses = 6 ;
            millis_jeda = millis();
        
      }
      if(nilai_pintu == LOW)
      {
        point_error = 6;
        point_menu = 8;
        flag_error_heater = false;
        point_proses = 8;
        loop_run =0;
        flag_start = false;
        millis_error = millis();
        
      }
    break;
    case 10: // 10. ur setelah st
      //Serial.println("10.Ur");
      digitalWrite(pB0,HIGH);
      digitalWrite(pB1,HIGH);
      digitalWrite(pB2,LOW);
      digitalWrite(pB3,LOW);
      digitalWrite(pB4,LOW);
      lc.setRow(1,0,B00111110);
      lc.setRow(1,1,B00000101);
      //pompa();
       if(pressure_value <0)
       {
         int nilai = abs(pressure_value);
         if(point_menu == 7 )
         {
          if(nilai > 81)
          {
            menit_run = waktu_drying ;
            millis_menit = millis();
            point_proses = 6 ;
            millis_jeda = millis();
           // Serial.println("2.Ur");
          }
         }else{
          if(nilai > 41)
          {
            menit_run = waktu_drying ;
            millis_menit = millis();
            point_proses = 6 ;
            millis_jeda = millis();
           // Serial.println("2.Ur");
          }
         }
         
       }
      if(nilai_pintu == LOW)
      {
        point_error = 6;
        point_menu = 8;
        flag_error_heater = false;
        point_proses = 8;
        loop_run =0;
        flag_start = false;
        millis_error = millis();
        
      }
      //point_proses = 3 ;
    break;

    
  default:
    // statements
   break;
  }
  
 // delay(2000);
}

// ======================= persiapan / LD ============================//
void ld()
{
  lc.setRow(1,1,B00111101);
  lc.setRow(1,0,B00001110);
}

void heater()
{
  int nilai_heater = temp_c_heater2;
  if(  nilai_heater < 154 )
  {
    if(flag_heater == true)
    {
      digitalWrite(pD5,HIGH);
    }else{
      digitalWrite(pD5,LOW);
    }
  }else{
    digitalWrite(pD5,HIGH);
  }
  
}

void bandheater()
{
  int nilai_bandheater = temp_c_heater1;
  if(  nilai_bandheater < 145 )
  {
    if(flag_bandheater == true)
    {
      digitalWrite(pD4,HIGH);
    }else{
      digitalWrite(pD4,LOW);
    }
  }else{
    digitalWrite(pD4,HIGH);
  }
  
}

void control_heater()
{
  if(point_menu != 7)
  {
    if( point_proses == 1 || point_proses == 2 || point_proses == 3 || point_proses == 4 || point_proses == 5 )
    {
      int nilai_heater = temp_c_heater2;
      int nilai_bandheater = temp_c_heater1;
      
      if(nilai_heater < (control_suhu_heater - 3) )
      {
        flag_heater = false;
        flag_bandheater = true;
      }else if(nilai_heater >= control_suhu_heater && nilai_bandheater < (control_suhu_bandheater - 5 ) )
      {
        flag_heater = true;
        flag_bandheater = false;
      }else if(nilai_heater >= control_suhu_heater && nilai_bandheater >= control_suhu_bandheater )
      {
         flag_heater = true;
         flag_bandheater = true;
      }
    }else if( point_proses == 6 || point_proses == 9 )
    {
      int nilai_heater = temp_c_heater2;
      int nilai_bandheater = temp_c_heater1;
      if(nilai_bandheater < 130 )
      {
        flag_heater = true;
        flag_bandheater = false;
      }else if(nilai_bandheater >= 135 )
      {
        flag_heater = true;
        flag_bandheater = false;
      }
    }else{
        flag_heater = true;
        flag_bandheater = true;
    }
  }
  
  vacuum_ur();
}

// ========================= POMPA =================================//
void pompa()
{
  //Serial.println("Program Pompa ");
  if(millis()- millis_pompa > millis_flagspompa)
  {
    if(millis()- millis_panas > millis_flagspanas)
    { 
      int nilai_heater = temp_c_heater2;
      
      if(nilai_heater > 140 )
      {
        delay(1000);
        digitalWrite(pD2,LOW);
        digitalWrite(pB2,HIGH);
        delay(250);
          
      }
      digitalWrite(pD2,HIGH);
      digitalWrite(pB2,LOW);
      jumlah_loop_panas = jumlah_loop_panas + 1;
      if( point_proses == 5)
      {
        if(jumlah_loop_panas > 3 )
        {
          jumlah_loop_panas = 0;
          millis_panas = millis();
        }   
      }else{
        if(jumlah_loop_panas > 5 )
        {
          jumlah_loop_panas = 0;
          millis_panas = millis();
        }
      }
      
      
    }
  }else{
    digitalWrite(pD2,HIGH);
    digitalWrite(pB2,LOW);
    
  }
}
// ======================= warm up / he ============================//
void warmup_he()
{
  DateTime now = rtc.now();
  if(temp_c_heater2 >= 90.00 && temp_c_heater1 >= 70.00 )
    {
        Serial.print(now.hour(), DEC); //jam
          Serial.print("."); 
          Serial.print(now.minute(), DEC); //menit
          Serial.print("  "); 
          Serial.print(temp_c);
          Serial.print("C   "); // SUHU
          Serial.print(pressure_value);
          Serial.print("KPa  "); //TEKANAN
            Serial.println("HE"); // MODE Pemanasan  
          
        delay(300);
        millis_flagserror = 600000;
        millis_error = millis();
         millis_panas = millis();
         millis_pompa = millis();
        point_proses = 2 ;
        delay(500);
    }
}

// ======================= vacuum / Ur ============================//
void vacuum_ur()
{
  if(point_proses == 2 || point_proses == 10 || point_proses == 6 && point_menu != 7)
  {
    digitalWrite(pD6,LOW);
    
  }else{
    digitalWrite(pD6,HIGH);
  }
}


void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);

  pinMode(pB0,OUTPUT); // selenoid 1
  pinMode(pB1,OUTPUT); // selenoid 2
  pinMode(pB2,OUTPUT); // selenoid 3
  pinMode(pB3,OUTPUT); // selenoid 4
  pinMode(pB4,OUTPUT); // selenoid 5
  pinMode(pA0,OUTPUT); 
  pinMode(pA1,OUTPUT); // Air
  pinMode(pA2,OUTPUT); // Limbah
  pinMode(pA3,OUTPUT); // Pintu
  pinMode(pA4,INPUT_PULLUP); // MCU MPX
  pinMode(pA5,INPUT_PULLUP); // Temperature heater
  pinMode(pA6,INPUT_PULLUP); // Temperature band heater
  pinMode(pA7,INPUT_PULLUP); // Temperature 

  pinMode(pD2,OUTPUT); // POMPA
  pinMode(pD3,OUTPUT); // XX
  pinMode(pD4,OUTPUT); // BANDHEATER
  pinMode(pD5,OUTPUT); // HEATER
  pinMode(pD6,OUTPUT); // VAC
  pinMode(pD7,OUTPUT); // FAN
  pinMode(pC6,INPUT_PULLUP); // INPUT MUC
  
  //pinMode(pD6, INPUT_PULLUP); // input muc .
  digitalWrite(pA3,LOW);
  digitalWrite(pA2,LOW);
  digitalWrite(pA1,LOW);
  digitalWrite(pA0,LOW);
  digitalWrite(pB0,LOW);
  digitalWrite(pB1,LOW);
  digitalWrite(pB2,LOW);
  digitalWrite(pB3,LOW);
  digitalWrite(pB4,LOW);
  digitalWrite(pD2,HIGH);
  digitalWrite(pD3,HIGH);
  digitalWrite(pD4,HIGH);
  digitalWrite(pD5,HIGH);
  digitalWrite(pD6,HIGH);
  digitalWrite(pD7,LOW);
  
  delay(500);  
  for(int k=0; k<2; k++){
    lc.shutdown(k,false);  // Enable display
    lc.setIntensity(k,5); // Set brightness level (0 is min, 15 is max)
    lc.clearDisplay(k);    // Clear display register
    delay(100);
  }
  delay(500);  

  int nilai_pintu = digitalRead(pA3);
  int nilai_air_kotor = digitalRead(pA2);
  int nilai_air_bersih = digitalRead(pA1);
   
  if(nilai_pintu == LOW)
  {
    bitWrite(nilai_lampu2, 6, 1); // lampu door
  }else{
    bitWrite(nilai_lampu2, 6, 0); // lampu door
  }
  
  for(int i=0;i<9;i++) {
    nilai_pintu = digitalRead(pA3);
    nilai_air_kotor = digitalRead(pA2);
    nilai_air_bersih = digitalRead(pA1);
    lc.setDigit(0,7,i,false);
    lc.setDigit(0,6,i,false);
    lc.setDigit(0,5,i,false);
    lc.setDigit(0,4,i,false);
    lc.setDigit(0,3,i,false);
    lc.setDigit(0,2,i,false);
    lc.setDigit(0,1,i,false);
    lc.setDigit(0,0,i,false);
    lc.setDigit(1,1,i,false);
    lc.setDigit(1,0,i,false);
    
    int n = 1;
    bitWrite(nilai_lampu, 0, n); // lampu katun
    bitWrite(nilai_lampu, 1, n); // lampu bd
    bitWrite(nilai_lampu, 2, n); // lampu 134 t
    bitWrite(nilai_lampu, 3, n); // lampu 134 b
    bitWrite(nilai_lampu, 4, n); // lampu 121 t
    bitWrite(nilai_lampu, 5, n); // lampu 121 b
    bitWrite(nilai_lampu, 6, n); // lampu error
    bitWrite(nilai_lampu, 7, n); // cicle
    bitWrite(nilai_lampu2, 4, n);
    bitWrite(nilai_lampu2, 5, n);
    bitWrite(nilai_lampu2, 6, n);
  
    lc.setRow(1,2,nilai_lampu);
    lc.setRow(1,3,nilai_lampu2);
    //lc.clearDisplay(0);

    if(nilai_air_kotor == HIGH || nilai_air_bersih == HIGH) 
    {
      flag_alarm_buzzer = true;
      
    
    }else{
      flag_alarm_buzzer = false;
      //bitWrite(nilai_lampu2, 4, 0); // lampu AIR bersih mati
    }
  buzzer();
  delay(1000);
  }
  nilai_lampu = B00000000;
  nilai_lampu2 = B00000000;
  lc.setRow(1,2,nilai_lampu);
  lc.setRow(1,3,nilai_lampu2);
  millis_baca_sensor = millis();
  millis_buzzer = millis();
    pembacaan_suhu_ruang();
    pembacaan_pressure();
    pembacaan_suhu_bandheater();
    pembacaan_suhu_heater();
  point_menu = 1;
  millis_serialprint = millis();
  millis_alarm = millis();
  millis_display_suhu = millis();
  if (! rtc.begin()) {
  
  }
  if (rtc.lostPower()) {
    //atur waktu sesuai waktu pada komputer
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  
  }
  
  millis_alarm_tangki = millis();
  //read nilai pada eeprom
  
  eeprom.eeprom_read(0, &var_suhu_ruang);
  eeprom.eeprom_read(3, &var_suhu_band);
  eeprom.eeprom_read(5, &var_suhu_heater);
  eeprom.eeprom_read(7, &var_pressure);
  eeprom.eeprom_read(9, &waktu_he);
  eeprom.eeprom_read(11, &waktu_ur);
  eeprom.eeprom_read(41, &waktu_re);
  eeprom.eeprom_read(13, &waktu_dn);

  if (waktu_he < 1 ) {
    waktu_he = 10;
   } 
   if (waktu_ur < 1) {
    waktu_ur = 8;
    } 

   if (waktu_re < 1) {
    waktu_re = 16;
   } 

  

   if (waktu_dn < 1) {
    waktu_dn = 5;
    } 
   
   
}

void input_button()
{
  for (int i = 0; i < 9; i++) {
        
        my_mux.channel(i);
        int s = digitalRead(pC6);
        if(s == 0 )
        {
          switch (i) {
            case 0: // 121 terbuka
              // statements
              if(flag_start == false && flag_setting == false)
              {
                if(point_menu != 8 )
                {
                  point_menu = 1;  
                }
                
               
              }else if(flag_start == false && flag_setting == true)
              { 
                
              }else{
                digitalWrite(pA0,HIGH);
                delay(200);
                digitalWrite(pA0,LOW);
              }
              
              break;
            case 1:// 121 tertutup
              // statements
              
              if(flag_start == false)
              {
                if(point_menu != 8 )
                {
                  point_menu = 2;  
                }  
                
              }else if(flag_start == false && flag_setting == true)
              { 
                
              }else{
                digitalWrite(pA0,HIGH);
                delay(200);
                digitalWrite(pA0,LOW);
              }
             // flag_tes_vacuum == false;
              
              break;
            case 2:// 134 terbuka
              // statements
              
              if(flag_start == false)
              {
                if(point_menu != 8 )
                {
                  point_menu = 3;  
                } 
               
              }else if(flag_start == false && flag_setting == true)
              { 
                
              }else{
                digitalWrite(pA0,HIGH);
                delay(200);
                digitalWrite(pA0,LOW);
              }
              
              break;
            case 3://134 tertutup
              
              if(flag_start == false)
              {
                if(point_menu != 8 )
                {
                  point_menu = 4;  
                } 
               
              }else if(flag_start == false && flag_setting == true)
              { 
                
              }else{
                digitalWrite(pA0,HIGH);
                delay(200);
                digitalWrite(pA0,LOW);
              }
              
              break;
            case 5:// 134 katun
              
              if(flag_start == false)
              {
                if(point_menu != 8 )
                {
                  point_menu = 5;  
                }  
               
              }else if(flag_start == false && flag_setting == true)
              { 
                
              }else{
                digitalWrite(pA0,HIGH);
                delay(200);
                digitalWrite(pA0,LOW);
              }
              
              break;
            case 4: // bd test
             
              //point_menu = 6;
              if(flag_start == false)
              {
                if(loop_tes_vacuum >= 2 )
                {
                  if(point_menu != 8 )
                  {
                    point_menu = 7;  
                  }
                }else{
                  if(point_menu != 8 )
                  {
                    point_menu = 6;  
                  }
                }
                if(millis()- millis_vacumtest > millis_flagvacumtest){
                  loop_tes_vacuum = loop_tes_vacuum + 1;  
                 // Serial.println(loop_tes_vacuum);
                  millis_vacumtest = millis();
                }
                if(loop_tes_vacuum == 5 )
                {
                  flag_tes_vacuum == true ;
                  point_menu = 7;
                  
                }  
               
              }else if(flag_start == false && flag_setting == true)
              { 
                
              }else{
                digitalWrite(pA0,HIGH);
                delay(200);
                digitalWrite(pA0,LOW);
              }
          
              break;
            case 6: // cycle
              if(millis()-millis_cycle > millis_flagcycle )
                  {
                    
                    if(flag_cycle == false)
                    {
                      bitWrite(nilai_lampu, 7, 1); // cicle
                      flag_cycle = true;  
                    }else{
                      bitWrite(nilai_lampu, 7, 0); // cicle
                      flag_cycle = false;
                    }
                    millis_cycle = millis();
                  }
                 
              
              break;
            case 7:// start
             if(flag_setting == false)
             {
                if(point_menu == 7)
                {
                   point_proses = 10;
                }else{
                   point_proses = 1;
                }
             
              flag_start = true;
             }else{
              if(hold_add_rtc == false)
              {
                data_waktu = data_waktu + 1;
                hold_add_rtc = true;
              }
              
             }
              
              break;
            case 8:// stop
              if(flag_start == true)
              {
                flag_setting = false;
                if(millis()- millis_stop > millis_flagstop){
                  loop_stop = loop_stop + 1;  
                  //Serial.println(loop_stop);
                  millis_stop = millis();
                }
                if(loop_stop == 2 )
                {
                  point_proses = 0;
                  
                  flag_start = false;
                   flag_setting = false;
                }  
              }else{
                if(flag_delay_setting == false )
                {
                
                    if(flag_setting == true)
                    {
                      loop_setting = loop_setting + 1;
                      change_add_rtc = false;
                    }
                  flag_delay_setting = true;
                }else{
                  
                  
                }
                
                if(millis()- millis_setting > millis_flagsetting){
                  point_menu = 9;
                  delay(2000);
                  if(flag_setting == false)
                  {
                    flag_setting = true;
                    loop_setting = 0;
                    
                    
                    
                  }else{
                    
                    if(flagSetRTC == false)
                    {
                      getTime();
                      flagSetRTC = true;  
                    }
                    
                   loop_setting = 15;
                  }
                  
                  millis_setting = millis();
                }else{
                }

              } 
              
              
              break;          
            default:
              // statements
              break;
          }  
        }else{
          switch (i) {
            case 4:
            loop_tes_vacuum = 0 ;
            millis_vacumtest = millis();
            break;
            case 7:
            hold_add_rtc = false;
            break;
            case 8:
            loop_stop = 0 ;
            flag_delay_setting = false ;
            millis_stop = millis();
            millis_setting = millis();
            break;           
            default:
              // statements
              break;
        }
    }
  }
}

void menu()
{
  switch (point_menu) {
  case 0: // set awal autoclave di jalankan
    ld();
    point_proses = 0;
    break;
  case 1: // 121 C terbuka
    ld();
    set_suhu_menu = 121; // satuan Celcius
    set_pressure_menu = 110; // satuan kpa
    waktu_st =20; // satuan menit
    waktu_drying = 10; // satuan menit
    control_suhu_heater = 152;
    control_suhu_bandheater = 120;
    //proses();
    bitWrite(nilai_lampu, 0, 0); // lampu katun
    bitWrite(nilai_lampu, 1, 0); // lampu bd
    bitWrite(nilai_lampu, 2, 0); // lampu 134 t
    bitWrite(nilai_lampu, 3, 0); // lampu 134 b
    bitWrite(nilai_lampu, 4, 0); // lampu 121 t
    bitWrite(nilai_lampu, 5, 1); // lampu 121 b
    bitWrite(nilai_lampu, 6, 0); // lampu error
    jml_loop = 2;
   
    
    break;
  case 2: // 121 C terbungkus
    ld();
    set_suhu_menu = 121; // satuan Celcius
    set_pressure_menu = 110; // satuan kpa
    waktu_st = 20; // satuan menit
    waktu_drying = 15; // satuan menit
    control_suhu_heater = 152;
    control_suhu_bandheater = 120;
    //proses();
    bitWrite(nilai_lampu, 0, 0); // lampu katun
    bitWrite(nilai_lampu, 1, 0); // lampu bd
    bitWrite(nilai_lampu, 2, 0); // lampu 134 t
    bitWrite(nilai_lampu, 3, 0); // lampu 134 b
    bitWrite(nilai_lampu, 4, 1); // lampu 121 t
    bitWrite(nilai_lampu, 5, 0); // lampu 121 b
    bitWrite(nilai_lampu, 6, 0); // lampu error
    jml_loop = 3;
    
    break;
  case 3: // 134C terbuka
    ld();
    set_suhu_menu = 134; // satuan Celcius
    set_pressure_menu = 210; // satuan kpa
    control_suhu_heater = 152;
    control_suhu_bandheater = 140;
    
    waktu_st = 4; // satuan menit
    waktu_drying = 4; // satuan menit
    //proses();
    bitWrite(nilai_lampu, 0, 0); // lampu katun
    bitWrite(nilai_lampu, 1, 0); // lampu bd
    bitWrite(nilai_lampu, 2, 0); // lampu 134 t
    bitWrite(nilai_lampu, 3, 1); // lampu 134 b
    bitWrite(nilai_lampu, 4, 0); // lampu 121 t
    bitWrite(nilai_lampu, 5, 0); // lampu 121 b
    bitWrite(nilai_lampu, 6, 0); // lampu error
    jml_loop = 2;
   
    break;
  case 4: // 134C terbungkus
    ld();
    set_suhu_menu = 134; // satuan Celcius
    set_pressure_menu = 210; // satuan kpa
    waktu_st = 4; // satuan menit
    waktu_drying = 15; // satuan menit
    control_suhu_heater = 152;
    control_suhu_bandheater = 140;
    
    jml_loop = 3;
    bitWrite(nilai_lampu, 0, 0); // lampu katun
    bitWrite(nilai_lampu, 1, 0); // lampu bd
    bitWrite(nilai_lampu, 2, 1); // lampu 134 t
    bitWrite(nilai_lampu, 3, 0); // lampu 134 b
    bitWrite(nilai_lampu, 4, 0); // lampu 121 t
    bitWrite(nilai_lampu, 5, 0); // lampu 121 b
    bitWrite(nilai_lampu, 6, 0); // lampu error
    break;
  case 5: // 134C katun
    set_suhu_menu = 134; // satuan Celcius
    set_pressure_menu = 210; // satuan kpa
    waktu_st = 4; // satuan menit
    waktu_drying = 20; // satuan menit
    control_suhu_heater = 152;
    control_suhu_bandheater = 140;
    ld();
    bitWrite(nilai_lampu, 0, 1); // lampu katun
    bitWrite(nilai_lampu, 1, 0); // lampu bd
    bitWrite(nilai_lampu, 2, 0); // lampu 134 t
    bitWrite(nilai_lampu, 3, 0); // lampu 134 b
    bitWrite(nilai_lampu, 4, 0); // lampu 121 t
    bitWrite(nilai_lampu, 5, 0); // lampu 121 b
    bitWrite(nilai_lampu, 6, 0); // lampu error
    jml_loop = 3;
    break;
  case 6: // BD test
    set_suhu_menu = 134; // satuan Celcius
    set_pressure_menu = 210; // satuan kpa
    control_suhu_heater = 152;
    control_suhu_bandheater = 140;
    waktu_st = 4; // satuan menit
    waktu_drying = 10; // satuan menit
    lc.setRow(1,1,B00111101);
    lc.setRow(1,0,B00011111);
    bitWrite(nilai_lampu, 0, 0); // lampu katun
    bitWrite(nilai_lampu, 1, 1); // lampu bd
    bitWrite(nilai_lampu, 2, 0); // lampu 134 t
    bitWrite(nilai_lampu, 3, 0); // lampu 134 b
    bitWrite(nilai_lampu, 4, 0); // lampu 121 t
    bitWrite(nilai_lampu, 5, 0); // lampu 121 b
    bitWrite(nilai_lampu, 6, 0); // lampu error
    jml_loop = 2;
    break;
  case 7: // vacuum test
    set_suhu_menu = 100; // satuan Celcius
    set_pressure_menu = -81; // satuan kpa
    point_proses = 10;
    waktu_st = 0; // satuan menit
    waktu_drying = 15; // satuan menit
    lc.setRow(1,1,B00111110);
    lc.setRow(1,0,B01110111);
    bitWrite(nilai_lampu, 0, 0); // lampu katun
    bitWrite(nilai_lampu, 1, 1); // lampu bd
    bitWrite(nilai_lampu, 2, 0); // lampu 134 t
    bitWrite(nilai_lampu, 3, 0); // lampu 134 b
    bitWrite(nilai_lampu, 4, 0); // lampu 121 t
    bitWrite(nilai_lampu, 5, 0); // lampu 121 b
    bitWrite(nilai_lampu, 6, 0); // lampu error
    jml_loop = 1;
    nilai_vacuum = 0;
    break;
  case 8:
    error();
    flag_print = false ;
      
    bitWrite(nilai_lampu, 6, 1); // lampu error
    break;
  case 9:
    update_jam();
    break;
  default:
    // statements
   break;
  }
  if(point_menu == 8 )
  {
    if(millis()-millis_alarm > millis_flagsalarm)
    {
      digitalWrite(pA0,HIGH);
      delay(200);
      digitalWrite(pA0,LOW);
      millis_alarm = millis();  
    }
    
    
  }else{
  
  }
}

void error()
{
  switch (point_error) {
    case 1: // E1 sensor temperatur bermasalah
      lc.setRow(1,0,B01001111);
      lc.setDigit(1,1,1,false);
      //lc.setRow(1,2,B01000000);
      break;
    case 2: // E2 kelebihan tekanan
      lc.setRow(1,0,B01001111);
      lc.setDigit(1,1,2,false);
     // lc.setRow(1,2,B01000000);
      break;
    case 3: // E3 overheat temperature dalam / heater
      lc.setRow(1,0,B01001111);
      lc.setDigit(1,1,3,false);
     // lc.setRow(1,2,B01000000);
      break;
    case 4: // E4 kegagalan preheating
      lc.setRow(1,0,B01001111);
      lc.setDigit(1,1,4,false);
      //lc.setRow(1,2,B01000000);
      break;
    case 5: // E5 tekanan tidak dapat dikeluarkan
      lc.setRow(1,0,B01001111);
      lc.setDigit(1,1,5,false);
     // lc.setRow(1,2,B01000000);
      break;
    case 6: // E6 pintu masih terbuka
      lc.setRow(1,0,B01001111);
      lc.setDigit(1,1,6,false);
     // lc.setRow(1,2,B01000000);
      break;
    case 7: // E7 tekanan udara local terlalu rendah
      lc.setRow(1,0,B01001111);
      lc.setDigit(1,1,7,false);
      //lc.setRow(1,2,B01000000);
      break;
    case 8: // E8 tidak dapat menaikan tekanan
      lc.setRow(1,0,B01001111);
      lc.setDigit(1,1,8,false);
      //lc.setRow(1,2,B01000000);
      break;
    case 9: // E9 tidak dapat mempertahankan tekanan
      lc.setRow(1,0,B01001111);
      lc.setDigit(1,1,9,false);
     // lc.setRow(1,2,B01000000);
      break;   
    default:
      // statements
     break;
  }
  
}

void loop() {
  int nilai_pintu = digitalRead(pA3);
  int nilai_air_kotor = digitalRead(pA2);
  int nilai_air_bersih = digitalRead(pA1);
  
  if(nilai_pintu == LOW)
  {
    bitWrite(nilai_lampu2, 6, 1); // lampu door
  }else{
    bitWrite(nilai_lampu2, 6, 0); // lampu door
  }
  if(millis() - millis_baca_sensor>millis_flagsbacasensor)
  {
     pembacaan_suhu_ruang();
    pembacaan_pressure();
    pembacaan_suhu_bandheater();
    pembacaan_suhu_heater();
    millis_baca_sensor = millis();
  }
 
  input_button();
  if(millis()- millis_pintu < millis_flagspintu){
    if(nilai_pintu == LOW && flag_start == true)
    {
       flag_start = false;
    }
  }
  
 
  if(flag_start == false)
  {
    flag_selesai_print = false;
    millis_pintu = millis();
    millis_alarm_tangki = millis();
    millis_error = millis();
    loop_run =0;
    menu();
    flag_print = false;
    flag_hasil_vacum = false;
    millis_serialprint = millis();
    //digitalWrite(pD3,HIGH);
    digitalWrite(pD7,LOW);
    digitalWrite(pD3,HIGH);
    if(pressure_value >10 )
    {
        digitalWrite(pB0,HIGH);
        digitalWrite(pB1,LOW);
        digitalWrite(pB2,HIGH);
        digitalWrite(pB3,HIGH);
        digitalWrite(pB4,LOW);
    }else if(pressure_value < -3 ){
        digitalWrite(pB0,LOW);
        digitalWrite(pB1,LOW);
        digitalWrite(pB2,LOW);
        digitalWrite(pB3,LOW);
        digitalWrite(pB4,HIGH);
    }else{
        digitalWrite(pB0,LOW);
        digitalWrite(pB1,LOW);
        digitalWrite(pB2,LOW);
        digitalWrite(pB3,LOW);
        digitalWrite(pB4,LOW);
    }
    digitalWrite(pD6,HIGH);
    digitalWrite(pD5,HIGH);
    digitalWrite(pD4,HIGH);
    
    flag_heater = true;
    flag_bandheater = true;
  }else if(flag_start == true){
    digitalWrite(pD3,LOW);
    digitalWrite(pD7,HIGH);
      heater();
  bandheater();
  vacuum_ur();
  cetak_print();
    if(point_menu == 8)
    {
      
    }else{
     if( nilai_pintu == HIGH)
     {
        proses();
        control_heater();
     }
         
     proses();
      
    }
    
  }

  
  if(nilai_air_bersih == HIGH)
  {
    flag_alarm_buzzer = true;
   if(millis() - millis_alarm_tangki > millis_flagalarm_tangki)
   {
     bitWrite(nilai_lampu2, 4, 1); // lampu AIR bersih menyala
     delay(200);
     bitWrite(nilai_lampu2, 4, 0); // lampu AIR bersih menyala
     millis_alarm_tangki = millis();
   }else{
    bitWrite(nilai_lampu2, 4, 1); // lampu AIR bersih menyala
    
   }
    /*bitWrite(nilai_lampu2, 4, 1); // lampu AIR bersih menyala
    loop_run =0;
    flag_start = false;*/
  }else{
    bitWrite(nilai_lampu2, 4, 0); // lampu AIR bersih menyala
  }
  
  if(nilai_air_kotor == HIGH) 
  {
    flag_alarm_buzzer = true;
   if(millis() - millis_alarm_tangki > millis_flagalarm_tangki)
   {
     bitWrite(nilai_lampu2, 5, 1); // lampu AIR bersih menyala
     delay(200);
     bitWrite(nilai_lampu2, 5, 0); // lampu AIR bersih menyala
     millis_alarm_tangki = millis();
   }else{
    bitWrite(nilai_lampu2, 5, 1); // lampu air kotor menyala
    
   } 
    
  }else{
    bitWrite(nilai_lampu2, 5, 0); // lampu air kotor mati
    if(nilai_air_bersih == HIGH)
    {
       flag_alarm_buzzer = true;
    }else{
      flag_alarm_buzzer = false;
    }
  }
 
  
  lc.setRow(1,2,nilai_lampu);
  lc.setRow(1,3,nilai_lampu2);
  if(flag_delay_setting == false)
  {
   // Serial.println("flag_delay_setting = false");
  }else{
  //  Serial.println("flag_delay_setting = true");
  }
  if(flag_setting == false)
  {
    //Serial.println("flag_setting = false;");
  }else{
   // Serial.println("flag_setting = true;");
  }    
  buzzer();
}

void cetak_print()
{
   DateTime now = rtc.now();
  if(flag_print == false && flag_start == true)
  {
    Serial.println("           AUTOCLAVE");
    Serial.println("            BARTEC");
    Serial.println("==============================");
    Serial.print("Tgl         : ");
    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);//hari
    Serial.print(", ");
    Serial.print(now.day(), DEC); //tanggal
    Serial.print('/');
    Serial.print(now.month(), DEC); //bulan
    Serial.print('/');
    Serial.print(now.year(), DEC); //tahun
    Serial.print(' ');
    Serial.print(now.hour(), DEC); //jam
    Serial.print(':');
    Serial.print(now.minute(), DEC); //tanggal
    Serial.print(':');
    Serial.print(now.second(), DEC); //detik
    Serial.println();
    Serial.print("MODE        : ");
    if(point_menu == 1 )
    {
      Serial.println("121 C / Terbuka");  
    }else if(point_menu == 2 )
    {
      Serial.println("121 C / Tertutup");  
    }else if(point_menu == 3 )
    {
      Serial.println("134 C / Terbuka");  
    } else if(point_menu == 4 )
    {
      Serial.println("134 C / Tertutup");  
    }else if(point_menu == 5 )
    {
      Serial.println("134 C / Kain");  
    }else if(point_menu == 6 )
    {
      Serial.println("134 C / BD TEST");  
    }else if(point_menu == 7 )
    {
      Serial.println("Vacuum Test");  
    }
    if(point_menu == 7 )
    {
    Serial.print("Pressure    : ");
    Serial.print(set_pressure_menu);
    Serial.println(" KPa");
    Serial.print("Time Dry    : ");
    Serial.print(waktu_drying);
    Serial.println(" M");
    }else{
      Serial.print("Pressure    : ");
    Serial.print(set_pressure_menu);
    Serial.println(" KPa");
    Serial.print("Temperature : ");
    Serial.print(set_suhu_menu);
    Serial.println(" C");
    Serial.print("Time St     : ");
    Serial.print(waktu_st);
    Serial.println(" M");
    Serial.print("Time Dry    : ");
    Serial.print(waktu_drying);
    Serial.println(" M");
    Serial.print("Pulse Cicle : ");
    Serial.println(jml_loop);
    
    }
    Serial.println("");
    Serial.println("TIME   TEMP     PRESS   MODE  ");
    Serial.print(now.hour(), DEC); //jam
          Serial.print("."); 
          Serial.print(now.minute(), DEC); //menit
          Serial.print("  "); 
          Serial.print(temp_c);
          Serial.print("C   "); // SUHU
          Serial.print(pressure_value);
          Serial.print("KPa  "); //TEKANAN
          Serial.println("WARM"); // MODE Pemanasan  
          
    
    millis_serialprint = millis();
      flag_print = true ;  
      
    
  }else if(flag_print == true && flag_start == true)
  {
     if(millis()- millis_serialprint > millis_flagsserialprint){
      if(point_proses == 7)
      {
          Serial.print(now.hour(), DEC); //jam
          Serial.print("."); 
          Serial.print(now.minute(), DEC); //menit
          Serial.print("  "); 
          Serial.print(temp_c);
          Serial.print("C   "); // SUHU
          Serial.print(pressure_value);
          Serial.print("KPa  "); //TEKANAN
          Serial.println("LE"); // MODE Pemanasan  
          millis_serialprint = millis();    
    
      }else if(point_proses == 8 && flag_selesai_print == true)
      {
          Serial.print(now.hour(), DEC); //jam
          Serial.print("."); 
          Serial.print(now.minute(), DEC); //menit
          Serial.print("  "); 
          Serial.print(temp_c);
          Serial.print("C   "); // SUHU
          Serial.print(pressure_value);
          Serial.print("KPa  "); //TEKANAN
          Serial.println("ED"); // MODE Pemanasan  
          Serial.println();
          Serial.println();
          Serial.println();
          flag_selesai_print = false;
      
      }else{
        millis_serialprint = millis();
      }
      
      
     }
    
    
  }else{
    
  }
}


void buzzer()
{
  if(millis() - millis_buzzer > millis_flagsbuzzer )
  {
    if(flag_jeda_buzzer == false && flag_alarm_buzzer == true)
    {
      digitalWrite(pA0,HIGH);
      flag_jeda_buzzer = true;
    }else if(flag_jeda_buzzer == true && flag_alarm_buzzer == true)
    {
      digitalWrite(pA0,LOW);
      flag_jeda_buzzer = false;
    }else if(flag_alarm_buzzer == false)
    {
      digitalWrite(pA0,LOW);
      flag_jeda_buzzer = false;
    }
    millis_buzzer = millis();
  }
  
}

void led_air_bersih()
{
  if(millis() - millis_ledbersih_alarm > millis_flagsledbersih_alarm )
  {
    if(flag_jeda_alarm_ledbersih == false && flag_ledbersih_alarm == true)
    {
      bitWrite(nilai_lampu2, 4, 1); // lampu AIR bersih menyala
      flag_jeda_alarm_ledbersih = true;
    }else if(flag_jeda_alarm_ledbersih == true && flag_ledbersih_alarm == true)
    {
      
      flag_jeda_alarm_ledbersih = false;
    }else if(flag_ledbersih_alarm == false)
    {
      
      flag_jeda_alarm_ledbersih = false;
    }
    millis_ledbersih_alarm = millis();
  }
  
}

void led_air_kotor()
{
  if(millis() - millis_ledkotor_alarm > millis_flagsledkotor_alarm )
  {
    if(flag_jeda_alarm_ledkotor == false && flag_ledkotor_alarm == true)
    {
      bitWrite(nilai_lampu2, 4, 1); // lampu AIR bersih menyala
      flag_jeda_alarm_ledkotor = true;
    }else if(flag_jeda_alarm_ledkotor == true && flag_ledkotor_alarm == true)
    {
      
      flag_jeda_alarm_ledkotor = false;
    }else if(flag_ledkotor_alarm == false)
    {
      
      flag_jeda_alarm_ledkotor = false;
    }
    millis_ledkotor_alarm = millis();
  }
  
}

void update_jam()
{
  int satuan = tahun%10;
  int puluhan = (tahun/10)%10;
  switch (loop_setting) {
    case 0: // R1
      lc.setRow(0,0,B01000110);
      lc.setDigit(0,1,1,false);
      lc.setRow(0,2,B00000000);
      //lc.setRow(1,2,B01000000);
      break;
    case 1: // R2
      lc.setRow(0,0,B01000110);
      lc.setDigit(0,1,2,false);
      lc.setRow(0,2,B00000000);// lc.setRow(1,2,B01000000);
      break;
    case 2: // R3
      lc.setRow(0,0,B01000110);
      lc.setDigit(0,1,3,false);
      lc.setRow(0,2,B00000000);
      // lc.setRow(1,2,B01000000);
      break;
    case 3: // R4
      lc.setRow(0,0,B01000110);
      lc.setDigit(0,1,4,false);
      lc.setRow(0,2,B00000000);
      //lc.setRow(1,2,B01000000);
      break;
    case 4: // R5
      lc.setRow(0,0,B01000110);
      lc.setDigit(0,1,5,false);
      lc.setRow(0,2,B00000000);
      // lc.setRow(1,2,B01000000);
      break;
    case 5: // P1
      lc.setRow(0,0,B01100111);
      lc.setDigit(0,1,1,false);
      lc.setRow(0,2,B00000000);
      break;
    case 6: // P2
      lc.setRow(0,0,B01100111);
      lc.setDigit(0,1,2,false);
      lc.setRow(0,2,B00000000);
      break;
    case 7: // P3
      lc.setRow(0,0,B01100111);
      lc.setDigit(0,1,3,false);
      lc.setRow(0,2,B00000000);
      break;
    case 8: // P4
      lc.setRow(0,0,B01100111);
      lc.setDigit(0,1,4,false);
      lc.setRow(0,2,B00000000);
      break;   
    case 9: // P5
      lc.setRow(0,0,B01100111);
      lc.setDigit(0,1,5,false);
      lc.setRow(0,2,B00000000);
      break;   
    case 10: // P6
      lc.setRow(0,0,B01100111);
      lc.setDigit(0,1,6,false);
      lc.setRow(0,2,B00000000);
      break;   
    case 11: // P7
      lc.setRow(0,0,B01100111);
      lc.setDigit(0,1,7,false);
      lc.setRow(0,2,B00000000);
      break;   
    case 12: // P8
      lc.setRow(0,0,B01100111);
      lc.setDigit(0,1,8,false);
      lc.setRow(0,2,B00000000);
      break;   
    case 13: // P9
      lc.setRow(0,0,B01100111);
      lc.setDigit(0,1,9,false);
      lc.setRow(0,2,B00000000);
      break;   
    case 14: // D0
      
      break;   
    case 15: // D1
      lc.setRow(0,0,B00111101);
      lc.setDigit(0,1,1,false);
      lc.setRow(0,2,B00000000);
      //detik = now.second();
      if(change_add_rtc == false)
      {
        data_waktu = tahun;
        change_add_rtc = true;
      }else{
        tahun = data_waktu;
      }
      satuan = tahun%10;
      puluhan = (tahun/10)%10;
      
      lc.setRow(0,6,B00000000);
      lc.setDigit(0,4,satuan,false);
      lc.setDigit(0,3,puluhan,false);
      lc.setRow(0,5,B00000000);
      
      break;   
    case 16: // D2
      lc.setRow(0,0,B00111101);
      lc.setDigit(0,1,2,false);
      lc.setRow(0,2,B00000000);
      
      if(change_add_rtc == false)
      {
        data_waktu = bulan;
        change_add_rtc = true;
      }else{
       if(data_waktu>12)
        {
          data_waktu = 1;  
        }else{
          bulan = data_waktu;
        }
        
      }
            
      satuan = bulan%10;
      puluhan = (bulan/10)%10;
      
      lc.setRow(0,6,B00000000);
      lc.setDigit(0,4,satuan,false);
      lc.setDigit(0,3,puluhan,false);
      lc.setRow(0,5,B00000000);
      break;   
    case 17: // D3
      lc.setRow(0,0,B00111101);
      lc.setDigit(0,1,3,false);
      lc.setRow(0,2,B00000000);
      
      if(change_add_rtc == false)
      {
        data_waktu = tanggal;
        change_add_rtc = true;
      }else{
        if(bulan == 1 || bulan == 3 || bulan == 5 || bulan == 7 || bulan == 8 || bulan == 10 || bulan == 12)
        {
          if(data_waktu>31)
          {
            data_waktu = 1;  
          }else{
            tanggal = data_waktu;
          }  
        }else if(bulan == 2)
        {
          if(data_waktu>29)
          {
            data_waktu = 1;  
          }else{
            tanggal = data_waktu;
          }  
        }else{
          if(data_waktu>30)
          {
            data_waktu = 1;  
          }else{
            tanggal = data_waktu;
          }  
        }
        
        
      }
      
      satuan = tanggal%10;
      puluhan = (tanggal/10)%10;
      
      lc.setRow(0,6,B00000000);
      lc.setDigit(0,4,satuan,false);
      lc.setDigit(0,3,puluhan,false);
      lc.setRow(0,5,B00000000);
      break;   
    case 18: // D4
      lc.setRow(0,0,B00111101);
      lc.setDigit(0,1,4,false);
      lc.setRow(0,2,B00000000);
      if(change_add_rtc == false)
      {
        data_waktu = jam;
        change_add_rtc = true;
      }else{
        if(data_waktu>23)
        {
          data_waktu = 0;  
        }else{
          jam = data_waktu;
        }
        
      }
      satuan = jam%10;
      puluhan = (jam/10)%10;
      
      lc.setRow(0,6,B00000000);
      lc.setDigit(0,4,satuan,false);
      lc.setDigit(0,3,puluhan,false);
      lc.setRow(0,5,B00000000);
      break;   
    case 19: // D5
      lc.setRow(0,0,B00111101);
      lc.setDigit(0,1,5,false);
      lc.setRow(0,2,B00000000);
      if(change_add_rtc == false)
      {
        data_waktu = menit;
        change_add_rtc = true;
      }else{
        if(data_waktu>59)
        {
          data_waktu = 0;  
        }else{
          menit = data_waktu;
        }
        //menit = data_waktu;
      }
      satuan = menit%10;
      puluhan = (menit/10)%10;
      
      lc.setRow(0,6,B00000000);
      lc.setDigit(0,4,satuan,false);
      lc.setDigit(0,3,puluhan,false);
      lc.setRow(0,5,B00000000);
      break;   
    case 20: // D6
      lc.setRow(0,0,B00111101);
      lc.setDigit(0,1,6,false);
      lc.setRow(0,2,B00000000);
      if(change_add_rtc == false)
      {
        data_waktu = detik;
        change_add_rtc = true;
      }else{
        if(data_waktu>59)
        {
          data_waktu = 0;  
        }else{
          detik = data_waktu;
        }
        
      }
      satuan = detik%10;
      puluhan = (detik/10)%10;
      
      lc.setRow(0,6,B00000000);
      lc.setDigit(0,4,satuan,false);
      lc.setDigit(0,3,puluhan,false);
      lc.setRow(0,5,B00000000);
      break;   
    case 21: // E9 tidak dapat mempertahankan tekanan
      //loop_setting = 0;
      flagSetRTC=false;
      rtc.adjust(DateTime(tahun, bulan, tanggal, jam, menit, detik));
      flag_delay_setting = false;
      flag_setting = false;
      point_menu = 1;
      pembacaan_pressure();
      resetFunc(); //call reset
      break;   
    default:
      // statements
     break;
  }
  //Serial.print("nilai data_waktu = ");
  //Serial.println(data_waktu);
}

void getTime(){
  
      DateTime now = rtc.now();
      detik = now.second();
      menit = now.minute();
      jam = now.hour();
      tanggal = now.day();
      bulan = now.month();
      tahun = now.year();
     // suhu_board = rtc.getTemperature();
     
  }
