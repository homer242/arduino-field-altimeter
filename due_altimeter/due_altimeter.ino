/*
* Anthony V.
*
* Dependencies:
* - bme280 library
* - u8g2 library
* - simpleKalmanFilter library
* - ezbutton library
*
* Hardware:
* - Arduino DUE (because the floating number can be double precision
*                but does it make really a difference with simple precision?)
* - OLED Yellow&Blue Display 0.96(SSD1315) connected through I2C
* - BME 280 connected through SPI
* - 4 push buttons connected to 40, 41, 42 and 43 digital pin numbers
*
* Needs and goals:
* - compared height difference between near points
* - the software boots and acquires the current pressure while 10 seconds.
*   The pressure will be used as reference. It can be reset using the
*   new session button.
*/

#include "bme280.h"
#include <U8g2lib.h>
#include <SimpleKalmanFilter.h>
#include <ezButton.h>

#include <Wire.h>

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

#define BME280_CS_PIN          53

#define PUSHBTN_DEBOUNCE_MS    50
#define SWITCH_MODE_DELAY_MS 2000

#define SEALEVELPRESSURE_PA 101325.0f

/* do not change values below without modifying
 * code elsewhere.
 */
#define MODE_ABSOLUTE           0
#define MODE_RELATIVE           1

enum state {
  state_exception = 0U,
  state_acq,
  state_conf,
};

typedef unsigned int (*sm_handler)(void);

static unsigned int sm_exception(void);
static unsigned int sm_acq(void);
static unsigned int sm_conf(void);

static sm_handler sm_hdls[] = {
  sm_exception,
  sm_acq,
  sm_conf,
};

/*
 * SimpleKalmanFilter(e_mea, e_est, q);
 * e_mea: Measurement Uncertainty
 * e_est: Estimation Uncertainty
 * q: Process Noise
 */
SimpleKalmanFilter absPressureKalmanFilter(1, 1, 0.01);
SimpleKalmanFilter relPressureKalmanFilter(1, 1, 0.01);

/* Grove - OLED Yellow&Blue Display 0.96(SSD1315) */
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);

/* BME-280 */
bfs::Bme280 bme(&SPI, BME280_CS_PIN /* CS pin number */);

/* Push buttons */
ezButton buttonSet(40 /* pin number */);
ezButton buttonOk(41 /* pin number */);
ezButton buttonUp(42 /* pin number */);
ezButton buttonDown(43 /* pin number */);

ezButton *buttonsArray[] = { &buttonSet, &buttonOk, &buttonUp, &buttonDown };

/* Machina */
static struct {
  int mode;
  unsigned int curr_state;

  double sea_pres_ref;
  double rel_pres_ref;

  double last_estim_pres;
} priv;

static const char *mode_str(int mode)
{
  switch (mode) {
  case MODE_ABSOLUTE:
    return "absolute";
  case MODE_RELATIVE:
    return "relative";
  default:
    return "unknown";
  }
}

static sm_handler get_sm_hdl(unsigned int state)
{
  if (state >= ARRAY_SIZE(sm_hdls))
    return sm_hdls[state_exception];

  return sm_hdls[state];
}

double getAltitude( double pres, double pres_ref, double temp )
{
  double a1 = pow( ( pres_ref / pres ), ( 1 / 5.257 ) );
  double a = (a1 - 1) * ( temp + 273.15 ) / 0.0065f;
  return a;
}

void oled_display_conf_sealevel(double sea_pres_pa)
{
  char buf[16];

  u8g2.clearBuffer();                   // clear the internal memory
  u8g2.setFont(u8g2_font_luBIS08_tf);   // choose a suitable font

  /* line title */
  u8g2.drawStr(0,10,"Sea level pressure (Pa)");

  /* line */
  snprintf(buf, sizeof(buf), "%.02f", sea_pres_pa);
  u8g2.drawStr(16,50,buf);

  u8g2.sendBuffer();                    // transfer internal memory to the display
}

void oled_display_acq(int mode, double temp_c, double alti_m)
{
  char buf[16];

  u8g2.clearBuffer();                   // clear the internal memory
  u8g2.setFont(u8g2_font_luBIS08_tf);   // choose a suitable font

  /* line mode */
  u8g2.drawStr(0,10,mode_str(mode));

  /* line altitude / height */
  if (mode == MODE_RELATIVE) {
    u8g2.drawStr(16,10,"Height (m):");
  } else {
    u8g2.drawStr(16,10,"Altitude (m):");
  }

  snprintf(buf, sizeof(buf), "%.02f", alti_m);
  u8g2.drawStr(16,50,buf);

  /* line temp */
  u8g2.drawStr(32,10,"Temp (C):");

  snprintf(buf, sizeof(buf), "%.02f", temp_c);
  u8g2.drawStr(32,50,buf);

  u8g2.sendBuffer();                    // transfer internal memory to the display
}

static void oled_display_exception(void)
{
  u8g2.clearBuffer();                   // clear the internal memory
  u8g2.setFont(u8g2_font_luBIS08_tf);   // choose a suitable font
  u8g2.drawStr(0,10,"Err!");    // write something to the internal memory
  u8g2.sendBuffer();                    // transfer internal memory to the display
}

void setup() {
  unsigned int i;

  /* Serial monitor for showing status and data */
  Serial.begin(115200);
  while (!Serial) {}
  /* Initialize the SPI bus */
  SPI.begin();
  /* Initialize OLED */
  u8g2.begin();
  /* Initialize push button controlers */
  for(i = 0; i < ARRAY_SIZE(buttonsArray); i++){
    buttonsArray[i]->setDebounceTime(PUSHBTN_DEBOUNCE_MS);
  }
  /* Initialize the BME-280 */
  if (!bme.Begin()) {
    Serial.println("Error initializing communication with BME-280");
    while (1) {}
  }

  priv.curr_state = state_acq;
  priv.sea_pres_ref = SEALEVELPRESSURE_PA;
}

static unsigned int sm_exception(void)
{
  oled_display_exception();

  return state_exception;
}

static unsigned int sm_acq(void)
{
  static unsigned long btn_ok_pressed_start_time;
  static int btn_ok_prev_is_pressed;

  unsigned long currentTime;
  double pres_pa,
    temp_c,
    hum_rh,
    alti_m;
  float estim_alti_m;
  int btn_ok_is_pressed;

  currentTime = millis();

  /***************************/
  /* acquisition and process */
  if (bme.Read()) {
    pres_pa = bme.pres_pa();
    temp_c = bme.die_temp_c();
    hum_rh = bme.humidity_rh();

    if (priv.mode == MODE_RELATIVE) {
      alti_m = getAltitude(pres_pa, priv.rel_pres_ref, temp_c);
      estim_alti_m = relPressureKalmanFilter.updateEstimate(alti_m);
    } else {
      alti_m = getAltitude(pres_pa, priv.sea_pres_ref, temp_c);
      estim_alti_m = absPressureKalmanFilter.updateEstimate(alti_m);
    }

    if (Serial.availableForWrite()) {
      Serial.print("pres(pa):");
      Serial.print(pres_pa);
      Serial.print(",");
      Serial.print("temp(c):");
      Serial.print(temp_c);
      Serial.print(",");
      Serial.print("humidity(rh):");
      Serial.print(hum_rh);
      Serial.print(",");
      Serial.print("alti(m):");
      Serial.print(alti_m);
      Serial.print(",");
      Serial.print("estim_alti(m):");
      Serial.println(estim_alti_m);
    }

    priv.last_estim_pres = estim_alti_m;

    oled_display_acq(priv.mode, temp_c, estim_alti_m);
  }

  /************************/
  /* handle button events */
  btn_ok_is_pressed = buttonOk.isPressed();
  if (btn_ok_is_pressed) {
    if (!btn_ok_prev_is_pressed) {
      btn_ok_pressed_start_time = currentTime;
      btn_ok_prev_is_pressed = true;
    }

    if (currentTime - btn_ok_pressed_start_time >= SWITCH_MODE_DELAY_MS) {
      /* switch mode */
      priv.mode = priv.mode ^ 1;

      /* update some values because of the new mode */
      if (priv.mode == MODE_RELATIVE) {
        priv.rel_pres_ref = priv.last_estim_pres;
      }

      /* start a new count */
      btn_ok_pressed_start_time = currentTime;
    }
  } else {
    btn_ok_prev_is_pressed = false;
  }

  if (buttonSet.isPressed()) {
    if (priv.mode == MODE_RELATIVE) {
      priv.rel_pres_ref = priv.last_estim_pres;
    } else {
      /* switch to config state */
      return state_conf;
    }
  }

  return state_acq;
}

static unsigned int sm_conf(void)
{
  /************************/
  /* handle button events */
  if (buttonOk.isPressed()) {
    return state_acq;
  }

  if (buttonUp.isPressed()) {
    priv.sea_pres_ref = floorf(priv.sea_pres_ref) + 1.0f;
  }

  if (buttonDown.isPressed()) {
    priv.sea_pres_ref = floorf(priv.sea_pres_ref) - 1.0f;
  }

  /************************/
  /* display */
  oled_display_conf_sealevel(priv.sea_pres_ref);

  return state_conf;
}

void loop() {
  sm_handler state_hdl;
  unsigned int i;

  /* MUST call the loop() function on ezbuttons before getting their status */
  for (i = 0; i < ARRAY_SIZE(buttonsArray); i++){
    buttonsArray[i]->loop();
  }

  state_hdl = get_sm_hdl(priv.curr_state);

  priv.curr_state = state_hdl();

  delay(100);
}
