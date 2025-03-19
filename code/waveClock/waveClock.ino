#include <lvgl.h>
#include <Wire.h>
#include <Arduino.h>
#include "pin_config.h"
#include "XPowersLib.h"
#include "lv_conf.h"
#include "Arduino_GFX_Library.h"
#include "Arduino_DriveBus_Library.h"
#include <ESP_IOExpander_Library.h>
#include "HWCDC.h"
#include "ui.h"
#include "ESP_I2S.h"
#include "esp_check.h"
#include "es8311.h"


#include "SensorPCF85063.hpp"

#define _EXAMPLE_CHIP_CLASS(name, ...) ESP_IOExpander_##name(__VA_ARGS__)
#define EXAMPLE_CHIP_CLASS(name, ...) _EXAMPLE_CHIP_CLASS(name, ##__VA_ARGS__)

int bri=120; // brightness
#define EXAMPLE_SAMPLE_RATE 16000
#define EXAMPLE_VOICE_VOLUME 80                // 0 - 100
#define EXAMPLE_MIC_GAIN (es8311_mic_gain_t)(3)  // 0 - 7
//#define EXAMPLE_RECV_BUF_SIZE (1000)

XPowersPMU power;
ESP_IOExpander *expander = NULL;
I2SClass i2s;


bool pmu_flag = 1;
bool adc_switch = false;

int sethour,setminute,setmonth,setday,setyear,setdow;
String percent="";
String voltage="";

int brightness[5]={80,120,160,200,255};
int bright=2;
String daysWeek[7]={"SUNDAY","MONDAY","TUESDAY","WEDNESDAY","THURSDAY","FRIDAY","SATURDAY"};

HWCDC USBSerial;
#define EXAMPLE_LVGL_TICK_PERIOD_MS 2

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[LCD_WIDTH * LCD_HEIGHT / 10];


SensorPCF85063 rtc;
uint32_t lastMillis;

Arduino_DataBus *bus = new Arduino_ESP32QSPI(
  LCD_CS /* CS */, LCD_SCLK /* SCK */, LCD_SDIO0 /* SDIO0 */, LCD_SDIO1 /* SDIO1 */,
  LCD_SDIO2 /* SDIO2 */, LCD_SDIO3 /* SDIO3 */);

Arduino_GFX *gfx = new Arduino_SH8601(bus, -1 /* RST */,
                                      0 /* rotation */, false /* IPS */, LCD_WIDTH, LCD_HEIGHT);

std::shared_ptr<Arduino_IIC_DriveBus> IIC_Bus =
std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);

void Arduino_IIC_Touch_Interrupt(void);
std::unique_ptr<Arduino_IIC> FT3168(new Arduino_FT3x68(IIC_Bus, FT3168_DEVICE_ADDRESS,
                                                       DRIVEBUS_DEFAULT_VALUE, TP_INT, Arduino_IIC_Touch_Interrupt));

const char *TAG = "esp32p4_i2s_es8311";

esp_err_t es8311_codec_init(void) {
  es8311_handle_t es_handle = es8311_create(0, ES8311_ADDRRES_0);
  ESP_RETURN_ON_FALSE(es_handle, ESP_FAIL, TAG, "es8311 create failed");
  const es8311_clock_config_t es_clk = {
    .mclk_inverted = false,
    .sclk_inverted = false,
    .mclk_from_mclk_pin = true,
    .mclk_frequency = EXAMPLE_SAMPLE_RATE * 256,
    .sample_frequency = EXAMPLE_SAMPLE_RATE
  };

  ESP_ERROR_CHECK(es8311_init(es_handle, &es_clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16));
  ESP_RETURN_ON_ERROR(es8311_sample_frequency_config(es_handle, es_clk.mclk_frequency, es_clk.sample_frequency), TAG, "set es8311 sample frequency failed");
  ESP_RETURN_ON_ERROR(es8311_microphone_config(es_handle, false), TAG, "set es8311 microphone failed");

  ESP_RETURN_ON_ERROR(es8311_voice_volume_set(es_handle, EXAMPLE_VOICE_VOLUME, NULL), TAG, "set es8311 volume failed");
  ESP_RETURN_ON_ERROR(es8311_microphone_gain_set(es_handle, EXAMPLE_MIC_GAIN), TAG, "set es8311 microphone gain failed");
  return ESP_OK;
}

void adcOn() {
  power.enableTemperatureMeasure();
  power.enableBattDetection();
  power.enableVbusVoltageMeasure();
  power.enableBattVoltageMeasure();
  power.enableSystemVoltageMeasure();
}

void adcOff() {
  power.disableTemperatureMeasure();
  power.disableBattDetection();
  power.disableVbusVoltageMeasure();
  power.disableBattVoltageMeasure();
  power.disableSystemVoltageMeasure();
}

   uint32_t getDayOfWeek(uint32_t day, uint32_t month, uint32_t year)
    {
        uint32_t val;
        if (month < 3) {
            month = 12u + month;
            year--;
        }
        val = (day + (((month + 1u) * 26u) / 10u) + year + (year / 4u) + (6u * (year / 100u)) + (year / 400u)) % 7u;
        if (0u == val) {
            val = 7;
        }
        return (val - 1);
    }

void Arduino_IIC_Touch_Interrupt(void) {
  FT3168->IIC_Interrupt_Flag = true;
}


/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif

  lv_disp_flush_ready(disp);
}

void example_increase_lvgl_tick(void *arg) {
  lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}



/*Read the touchpad*/
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
  int32_t touchX = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
  int32_t touchY = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);

  if (FT3168->IIC_Interrupt_Flag == true) {
    FT3168->IIC_Interrupt_Flag = false;
    data->state = LV_INDEV_STATE_PR;

    /*Set the coordinates*/
    data->point.x = touchX;
    data->point.y = touchY;

    
    gfx->Display_Brightness(bri);

    USBSerial.print("Data x ");
    USBSerial.print(touchX);

    USBSerial.print("Data y ");
    USBSerial.println(touchY);
  } else {
    data->state = LV_INDEV_STATE_REL;
  }
}

void setFlag(void) {
  pmu_flag = true;
}



void setup() {
  USBSerial.begin(115200); /* prepare for possible serial debug */
  pinMode(PA, OUTPUT),
  digitalWrite(PA, HIGH);

   if (!rtc.begin(Wire, PCF85063_SLAVE_ADDRESS, IIC_SDA, IIC_SCL)) {
    USBSerial.println("Failed to find PCF8563 - check your wiring!");
    while (1) {
      delay(1000);
    }
  }



  //rtc.setDateTime(year, month, day, hour, minute, second);

  i2s.setPins(BCLKPIN, WSPIN, DIPIN, DOPIN, MCLKPIN);
  if (!i2s.begin(I2S_MODE_STD, EXAMPLE_SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO, I2S_STD_SLOT_BOTH)) {
    Serial.println("Failed to initialize I2S bus!");
    return;
  }

  Wire.begin(IIC_SDA, IIC_SCL);

  es8311_codec_init();
  if (!rtc.begin(Wire, PCF85063_SLAVE_ADDRESS, IIC_SDA, IIC_SCL)) {
    USBSerial.println("Failed to find PCF8563 - check your wiring!");
    while (1) {
      delay(1000);
    }
  }


  expander = new EXAMPLE_CHIP_CLASS(TCA95xx_8bit,
                                    (i2c_port_t)0, ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000,
                                    IIC_SCL, IIC_SDA);

  bool result = power.begin(Wire, AXP2101_SLAVE_ADDRESS, IIC_SDA, IIC_SCL);

  if (result == false) {
    USBSerial.println("PMU is not online...");
    while (1) delay(50);
  }

  expander->init();
  expander->begin();
  expander->pinMode(5, INPUT);
  expander->pinMode(4, INPUT);
  expander->pinMode(1, OUTPUT);
  expander->pinMode(2, OUTPUT);
  expander->digitalWrite(1, LOW);
  expander->digitalWrite(2, LOW);
  delay(20);
  expander->digitalWrite(1, HIGH);
  expander->digitalWrite(2, HIGH);

  int pmu_irq = expander->digitalRead(5);
  if (pmu_irq == 1) {
    setFlag();
  }

  power.disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
  power.setChargeTargetVoltage(3);
  power.clearIrqStatus();
  power.enableIRQ(
    XPOWERS_AXP2101_PKEY_SHORT_IRQ  //POWER KEY
  );

  adcOn();

  while (FT3168->begin() == false) {
    USBSerial.println("FT3168 initialization fail");
    delay(2000);
  }
  USBSerial.println("FT3168 initialization successfully");

 

  gfx->begin();
  gfx->Display_Brightness(brightness[bright]);
  lv_init();

  FT3168->IIC_Write_Device_State(FT3168->Arduino_IIC_Touch::Device::TOUCH_POWER_MODE,
                                 FT3168->Arduino_IIC_Touch::Device_Mode::TOUCH_POWER_MONITOR);

  lv_disp_draw_buf_init(&draw_buf, buf, NULL, LCD_WIDTH * LCD_HEIGHT / 10);

  /*Initialize the display*/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  /*Change the following line to your display resolution*/
  disp_drv.hor_res = LCD_WIDTH;
  disp_drv.ver_res = LCD_HEIGHT;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  /*Initialize the (dummy) input device driver*/
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);

  const esp_timer_create_args_t lvgl_tick_timer_args = {
    .callback = &example_increase_lvgl_tick,
    .name = "lvgl_tick"
  };

  esp_timer_handle_t lvgl_tick_timer = NULL;
  esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer);
  esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000);

  ui_init();
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


     if(btn==ui_saveTime || btn==ui_saveDate )
     {
     rtc.setDateTime(setyear,setmonth, setday, sethour, setminute, 0);
     lv_scr_load(ui_Screen1);
     }
     
     }


void cngBrightness(lv_event_t * e)
{bright++;
  if(bright>4) bright=0;
  gfx->Display_Brightness(brightness[bright]);

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

void callBeep(lv_event_t * e)
{beep(2500,60);}

void brightMax(lv_event_t * e)
{gfx->Display_Brightness(255);}

void brightNormal(lv_event_t * e)
{gfx->Display_Brightness(brightness[bright]);}

void prepareTime(lv_event_t * e)
{
      RTC_DateTime datetime = rtc.getDateTime();
      sethour=datetime.hour;
      setminute=datetime.minute;  
      setyear=datetime.year-2000;
      setmonth=datetime.month;
      setday=datetime.day;
}


void beep(int frequency, int duration) {
    const int SAMPLE_RATE = 16000;
    const int AMPLITUDE = 8000;
    const float TWO_PI2 = 6.283185;
    long samples = (SAMPLE_RATE * duration) / 1000;
    unsigned char buffer[samples];

    for (int i = 0; i < samples; i++) {
        buffer[i] = AMPLITUDE * sin((TWO_PI2 * frequency * i) / SAMPLE_RATE);
    }

    size_t bytes_written;
    i2s.write((uint8_t *)buffer, samples);
    
}



void loop() {

  lv_timer_handler(); /* let the GUI do its work */
  delay(5);

    uint32_t status = power.getIrqStatus();
    if (power.isPekeyShortPressIrq()) {
        adcOff();
        power.shutdown();
    }
    power.clearIrqStatus();


    if (lv_scr_act() == ui_Screen1)
  {
   if (millis() - lastMillis > 1000) {
      lastMillis = millis();
    RTC_DateTime datetime = rtc.getDateTime();

    char buf[16];
    snprintf(buf, sizeof(buf), "%02d:%02d",datetime.hour, datetime.minute);
    char buf2[4];
    snprintf(buf2, sizeof(buf2), "%02d", datetime.second);
     char buf3[16];
    snprintf(buf3, sizeof(buf3), "%02d-%02d",datetime.day, datetime.month);

    // Update label with current time
    
    lv_label_set_text(ui_timeLBL, buf);
    lv_label_set_text(ui_secLBL, buf2);
    lv_label_set_text(ui_dateLBL, buf3);

    int dow=getDayOfWeek(datetime.day, datetime.month, datetime.year);
    lv_label_set_text(ui_dayLBL,daysWeek[dow].c_str());
  }

  percent=String(power.getBatteryPercent())+"%";
  lv_label_set_text(ui_batPercent,percent.c_str());  
  voltage=String(power.getBattVoltage())+"mV";
   lv_label_set_text(ui_batVoltage,voltage.c_str());
   lv_bar_set_value(ui_Bar1,percent.toInt(),LV_ANIM_OFF);

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
  }
}
