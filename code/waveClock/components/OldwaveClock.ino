/*Using LVGL with Arduino requires some extra steps:
 *Be sure to read the docs here: https://docs.lvgl.io/master/get-started/platforms/arduino.html  */

#include "Display_SPD2010.h"
#include "RTC_PCF85063.h"
#include "LVGL_Driver.h"
//#include "PWR_Key.h"
#include "RTC_PCF85063.h"
#include "ui.h"

#define BAT_ADC_PIN   8
#define Measurement_offset 0.990476 
#define MIN_VOLTAGE 3.2  // Voltage when the battery is empty
#define MAX_VOLTAGE 4.2  // Voltage when the battery is fully charged
int brightness[5]={20,40,60,80,100};
int bright=1;

String daysWeek[7]={"SUNDAY","MONDAY","TUESDAY","WEDNESDAY","THURSDAY","FRIDAY","SATURDAY"};

int h,s,m,y,mm,d,dow=0;
String hr,mi,se,da,mo,timeStr,dateStr,batStr,percentStr="";
float voltage,percent=0;

int sethour,setminute,setmonth,setday,setyear,setdow;

 float BAT_Get_Volts(void)
{
  int Volts = analogReadMilliVolts(BAT_ADC_PIN); // millivolts
  float BAT_analogVolts = (float)(Volts * 3.0 / 1000.0) / Measurement_offset;
  // printf("BAT voltage : %.2f V\r\n", BAT_analogVolts);
  return BAT_analogVolts;
}


float getBatteryPercentage(float voltage) {
    // Clamp the voltage to the range [MIN_VOLTAGE, MAX_VOLTAGE]
    if (voltage > MAX_VOLTAGE) voltage = MAX_VOLTAGE;
    if (voltage < MIN_VOLTAGE) voltage = MIN_VOLTAGE;

    // Calculate percentage
    float percentage = ((voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE)) * 100.0;
    return percentage;
}


void settTime(int y,int m,int d,int h,int mm,int s,int dow)
{datetime_t tt;
tt.year=y+2000;
tt.month=m;
tt.day=d;
tt.hour=h;
tt.minute=mm;
tt.second=s;
tt.dotw=dow;
PCF85063_Set_All(tt);
}

void setup()
{
  //Driver_Init();
   pinMode(7,OUTPUT);
   digitalWrite(7,1);
 
  LCD_Init();
  Backlight_Init();
  Set_Backlight(40);  
  I2C_Init();
  TCA9554PWR_Init(0x00);   
  PCF85063_Init();
  
  Lvgl_Init();
  ui_init();
  
}

void cngBright(lv_event_t * e)
{
  bright++;
  if(bright>4) bright=0;
  Set_Backlight(brightness[bright]); 

  if(bright==0)
  {
    lv_obj_add_flag(ui_Panel8,LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_Panel9,LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_Panel10,LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_Panel13,LV_OBJ_FLAG_HIDDEN);
  }
    if(bright==1)
  {
    lv_obj_clear_flag(ui_Panel8,LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_Panel9,LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_Panel10,LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_Panel13,LV_OBJ_FLAG_HIDDEN);
  }

      if(bright==2)
  {
    lv_obj_clear_flag(ui_Panel8,LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_Panel9,LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_Panel10,LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_Panel13,LV_OBJ_FLAG_HIDDEN);
  }

        if(bright==3)
  {
    lv_obj_clear_flag(ui_Panel8,LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_Panel9,LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_Panel10,LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_Panel13,LV_OBJ_FLAG_HIDDEN);
  }

          if(bright==4)
  {
    lv_obj_clear_flag(ui_Panel8,LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_Panel9,LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_Panel10,LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_Panel13,LV_OBJ_FLAG_HIDDEN);
  }


}

void prepareTime(lv_event_t * e)
{     
      sethour=h;
      setminute=m;  
      setyear=25;
      setmonth=mm;
      setday=d;
      setdow=dow;
}

void choseTime(lv_event_t * e)
{
     lv_obj_t * btn = lv_event_get_target(e);
     if(btn==ui_hrUp)
        {sethour++; if(sethour>23) sethour=0;}
     if(btn==ui_hrDown)
        {sethour--; if(sethour<0) sethour=23;}

    if(btn==ui_minDown)
     {setminute--; if(setminute<0) setminute=59;}
    if(btn==ui_minUp)
     {setminute++; if(setminute>59) setminute=0;}

        if(btn==ui_monthUP)
       {setmonth++; if(setmonth>12) setmonth=1;}
      if(btn==ui_monthDowm)
       {setmonth--; if(setmonth<1) setmonth=12;}

      if(btn==ui_dayUp)
       {setday++; if(setday>31) setday=1;}
      if(btn==ui_dayDown)
       {setday--; if(setday<1) setday=31;}

      if(btn==ui_yearUp)
       {setyear++; if(setyear>60) setyear=24;}
      if(btn==ui_yearDown)
       {setyear--; if(setyear<24) setyear=60;}

       if(btn==ui_dowUp)
       {dow++; if(dow>6) dow=0;}
      if(btn==ui_dowDown)
       {setdow--; if(setdow<0) setdow=0;}

     if(btn==ui_saveTime || btn==ui_saveDate )
     {
     settTime(setyear,setmonth,setday,sethour,setminute,0,setdow);
     lv_scr_load(ui_Screen1);
     }
     
     }

void loop() {

  if (lv_scr_act() == ui_Screen1)
  {
  PCF85063_Loop();
  s=datetime.second;
  m=datetime.minute;
  h=datetime.hour;
  d=datetime.day;
  y=datetime.year;
  mm=datetime.month;
  dow=datetime.dotw;

  if(s<10) se="0"+String(s); else se=String(s); 
  if(m<10) mi="0"+String(m); else mi=String(m); 
  if(h<10) hr="0"+String(h); else hr=String(h); 
  if(d<10) da="0"+String(d); else da=String(d); 
  if(mm<10) mo="0"+String(mm); else mo=String(mm); 
  timeStr=hr+":"+mi;
  dateStr=mo+"-"+da;

  voltage=BAT_Get_Volts();
  percent=getBatteryPercentage(voltage);
  batStr=String(voltage)+ " V";
  percentStr=String((int)percent)+"%";

  lv_label_set_text(ui_timeLBL,timeStr.c_str());
  lv_label_set_text(ui_secLBL,String(se).c_str());
  lv_label_set_text(ui_dateLBL,dateStr.c_str());
  lv_label_set_text(ui_dayLBL,daysWeek[dow].c_str());
  lv_label_set_text(ui_batVoltage,batStr.c_str());
  lv_label_set_text(ui_batPercent,percentStr.c_str());
  lv_bar_set_value(ui_Bar1,(int)percent,LV_ANIM_OFF);
  }

   if (lv_scr_act() == ui_setTime)
  {
    lv_label_set_text(ui_setHrLBL, String(sethour).c_str());
    lv_label_set_text(ui_setMinLBL, String(setminute).c_str()); 
  }

     if (lv_scr_act() == ui_setDate)
  {
    lv_label_set_text(ui_setMonthLBL, String(setmonth).c_str());
    lv_label_set_text(ui_setDayLBL, String(setday).c_str());
    lv_label_set_text(ui_setYearLBL, String(setyear).c_str());
    lv_label_set_text(ui_setDowLBL, String(setdow).c_str());
  }

  

  lv_timer_handler(); 
  vTaskDelay(pdMS_TO_TICKS(5));

}
