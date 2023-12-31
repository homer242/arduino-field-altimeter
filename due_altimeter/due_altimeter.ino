/*
* Anthony V.
*
* Dependencies:
* - bme280 library
* - ADAFRUIT bmp390 library (and its dependencies)
* - u8g2 library
* - simpleKalmanFilter library
* - ezbutton library
*
* Hardware:
* - Arduino DUE (because the floating number can be double precision
*                but does it make really a difference with simple precision?)
* - OLED Yellow&Blue Display 0.96(SSD1315) connected through I2C
* - BME 280 connected through SPI
* - BMP 390 connected through SPI
* - 4 push buttons connected to 40, 41, 42 and 43 digital pin numbers
*
* Needs and goals:
* - compared height difference between near points
* - the software boots and acquires the current pressure while 10 seconds.
*   The pressure will be used as reference. It can be reset using the
*   new session button.
*/

#include "bme280.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <U8g2lib.h>
#include <SimpleKalmanFilter.h>
#include <ezButton.h>

#include <Wire.h>

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

#define PUSHBUTTON_SET          4
#define PUSHBUTTON_OK           8
#define PUSHBUTTON_DOWN         6
#define PUSHBUTTON_UP           7

#define BMP390_CS_PIN          52
#define BME280_CS_PIN          53

#define PUSHBTN_DEBOUNCE_MS    50
#define SWITCH_MODE_DELAY_MS 2000
#define SWITCH_SNSR_DELAY_MS 2000

#define SEALEVELPRESSURE_PA 101325.0f

#define DT_FAST_MS             25
#define DT_NOMINAL_MS         100

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

/* Grove - OLED Yellow&Blue Display 0.96(SSD1315) */
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);

/* BME-280 */
bfs::Bme280 bme(&SPI, BME280_CS_PIN /* CS pin number */);

/* BMP390 */
Adafruit_BMP3XX bmp390;

/* Push buttons */
ezButton buttonSet(PUSHBUTTON_SET, INPUT);
ezButton buttonOk(PUSHBUTTON_OK, INPUT);
ezButton buttonUp(PUSHBUTTON_UP, INPUT);
ezButton buttonDown(PUSHBUTTON_DOWN, INPUT);

ezButton *buttonsArray[] = { &buttonSet, &buttonOk, &buttonUp, &buttonDown };

typedef int (*snsr_init)(void);
typedef int (*snsr_acq_do)(double *hum_rh, double *temp_c, double *press_pa);

struct snsr_drv {
  const char *name;
  bool init;
  struct {
    snsr_init init;
    snsr_acq_do acq;
  } ops;
};

static int bme280_init(void);
static int bme280_acq(double *hum_rh, double *temp_c, double *press_pa);

static struct snsr_drv bme280_drv = {
  .name = "bme280",
  .init = false,
  .ops = {
    .init = bme280_init,
    .acq = bme280_acq,
  },
};

static int bmp390_init(void);
static int bmp390_acq(double *hum_rh, double *temp_c, double *press_pa);

static struct snsr_drv bmp390_drv = {
  .name = "bmp390",
  .init = false,
  .ops = {
    .init = bmp390_init,
    .acq = bmp390_acq,
  }
};

static struct snsr_drv *snsr_drvs[] = {
  &bmp390_drv,
  &bme280_drv,
};

/* Machina */
static struct {
  unsigned int dt_ms;
  int mode;
  unsigned int curr_state;
  struct snsr_drv *curr_snsr_drv;

  double sea_pres_ref;

  double rel_alti_ref;

  double last_alti_m;
} priv;

/*
 * sensor drivers
 */
static int bme280_init(void)
{
  /* Initialize the BME-280 */
  if (!bme.Begin()) {
    Serial.println("Error initializing communication with BME-280");
    return -1;
  }

  return 0;
}

static int bme280_acq(double *hum_rh, double *temp_c, double *press_pa)
{
  if (!bme.Read())
    return -1;

  *hum_rh = bme.humidity_rh();
  *temp_c = bme.die_temp_c();
  *press_pa = bme.pres_pa();

  return 0;
}

static int bmp390_init(void)
{
  if (! bmp390.begin_SPI(BMP390_CS_PIN)) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    return -1;
  }

  // Set up oversampling and filter initialization
  bmp390.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp390.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  bmp390.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp390.setOutputDataRate(BMP3_ODR_25_HZ);

  return 0;
}

static int bmp390_acq(double *hum_rh, double *temp_c, double *press_pa)
{
  if (! bmp390.performReading())
    return -1;

  *hum_rh = 0;
  *temp_c = bmp390.temperature;
  *press_pa = bmp390.pressure;

  return 0;
}

/*
 *
 */
void oled_display_conf_sealevel(double sea_pres_pa)
{
  char buf[16];

  u8g2.clearBuffer();                   // clear the internal memory
  u8g2.setFont(u8g2_font_luBIS08_tf);   // choose a suitable font

  /* line title */
  u8g2.drawStr(0,10,"Sea level pressure (Pa)");

  /* line */
  snprintf(buf, sizeof(buf), "%.02f", sea_pres_pa);
  u8g2.drawStr(30,40,buf);

  u8g2.sendBuffer();                    // transfer internal memory to the display
}

void oled_display_acq(const struct snsr_drv *curr_drv, int mode,
                      double temp_c, double elevation_m)
{
  char buf[16];

  u8g2.clearBuffer();                   // clear the internal memory
  u8g2.setFont(u8g2_font_luBIS08_tf);   // choose a suitable font

  /* line mode + driver */
  u8g2.drawStr(0,10,mode_str(mode));
  u8g2.drawStr(64,10,curr_drv->name);

  /* line altitude / height */
  if (mode == MODE_RELATIVE) {
    u8g2.drawStr(0,30,"Height:");
  } else {
    u8g2.drawStr(0,30,"Alti:");
  }

  snprintf(buf, sizeof(buf), "%.02f m", elevation_m);
  u8g2.drawStr(60,30,buf);

  /* line temp */
  u8g2.drawStr(0,50,"Temp:");

  snprintf(buf, sizeof(buf), "%.02f C", temp_c);
  u8g2.drawStr(60,50,buf);

  u8g2.sendBuffer();                    // transfer internal memory to the display
}

static void oled_display_exception(const char *err_msg)
{
  u8g2.clearBuffer();                   // clear the internal memory
  u8g2.setFont(u8g2_font_luBIS08_tf);   // choose a suitable font
  u8g2.drawStr(0,10,"Err!");    // write something to the internal memory
  u8g2.drawStr(0,30,err_msg);    // write something to the internal memory
  u8g2.sendBuffer();                    // transfer internal memory to the display
}

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

static void switch_to_next_snsr(void)
{
  size_t nsnsrs;
  bool found_curr;
  unsigned int i,
    curr_snsr_idx;

  nsnsrs = 0;
  found_curr = false;
  for(i = 0; i < ARRAY_SIZE(snsr_drvs); i++) {
    if (!found_curr && priv.curr_snsr_drv == snsr_drvs[i]) {
      curr_snsr_idx = i;
      found_curr = true;
    }

    if (snsr_drvs[i]->init)
      nsnsrs += 1;
  }

  if (!found_curr) {
    oled_display_exception("no found snsr!");
    while (1) { }
  }

  if (nsnsrs < 2)
    return; /* nothing to do */

  /* take the next sensor */
  i = (curr_snsr_idx + 1) % nsnsrs;
  while (1) {
    if (snsr_drvs[i]->init) {
      priv.curr_snsr_drv = snsr_drvs[i];
      break;
    }

    i = (i + 1) % nsnsrs;
  }
}

double getAltitude( double pres, double pres_ref, double temp )
{
  double a1 = pow( ( pres_ref / pres ), ( 1 / 5.257 ) );
  double a = (a1 - 1) * ( temp + 273.15 ) / 0.0065f;
  return a;
}

void setup() {
  unsigned int i;

  priv.dt_ms = DT_NOMINAL_MS;

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
  /* Initialize sensors */
  priv.curr_snsr_drv = NULL;
  for(i = 0; i < ARRAY_SIZE(snsr_drvs); i++) {
    snsr_drvs[i]->init = (snsr_drvs[i]->ops.init() == 0);
    if (!snsr_drvs[i]->init) {
      Serial.print("Error initializing communication with ");
      Serial.println(snsr_drvs[i]->name);
    } else {
      if (priv.curr_snsr_drv == NULL)
        priv.curr_snsr_drv = snsr_drvs[i];
    }
  }

  if (priv.curr_snsr_drv == NULL) {
    oled_display_exception("no found sensors");
    while (1) {}
  }

  priv.curr_state = state_acq;
  priv.sea_pres_ref = SEALEVELPRESSURE_PA;
}

static unsigned int sm_exception(void)
{
  oled_display_exception("state machine error");

  priv.dt_ms = DT_NOMINAL_MS;

  return state_exception;
}

static unsigned int sm_acq(void)
{
  static unsigned long btn_ok_pressed_start_time;
  static int btn_ok_prev_is_pressed;
  static bool latchSwitchSnsr,
    switchSnsrSeqStarted;
  static unsigned long switch_snsr_seq_start_time;

  unsigned long currentTime;
  double pres_pa,
    temp_c,
    hum_rh,
    alti_m;
  float estim_alti_m,
    elevation_m;
  int btn_ok_is_pressed;

  if (priv.mode == MODE_RELATIVE) {
    priv.dt_ms = DT_FAST_MS;
  } else {
    priv.dt_ms = DT_NOMINAL_MS;
  }

  currentTime = millis();

  /***************************/
  /* acquisition and process */
  if (priv.curr_snsr_drv->ops.acq(&hum_rh, &temp_c, &pres_pa) == 0) {
    alti_m = getAltitude(pres_pa, priv.sea_pres_ref, temp_c);
    estim_alti_m = absPressureKalmanFilter.updateEstimate(alti_m);

    /* if (Serial.availableForWrite()) { */
    /*   Serial.print("pres(pa):"); */
    /*   Serial.print(pres_pa); */
    /*   Serial.print(","); */
    /*   Serial.print("temp(c):"); */
    /*   Serial.print(temp_c); */
    /*   Serial.print(","); */
    /*   Serial.print("humidity(rh):"); */
    /*   Serial.print(hum_rh); */
    /*   Serial.print(","); */
    /*   Serial.print("alti(m):"); */
    /*   Serial.print(alti_m); */
    /*   Serial.print(","); */
    /*   Serial.print("estim_alti(m):"); */
    /*   Serial.println(estim_alti_m); */
    /* } */

    if (priv.mode == MODE_RELATIVE) {
      elevation_m = estim_alti_m - priv.rel_alti_ref;
    } else {
      elevation_m = estim_alti_m;
    }

    priv.last_alti_m = estim_alti_m;

    oled_display_acq(priv.curr_snsr_drv, priv.mode, temp_c, elevation_m);
  }

  /************************/
  /* handle button events */

  /* OK button => switch mode */
  btn_ok_is_pressed = (buttonOk.getState() == 0);

  if (btn_ok_is_pressed) {
    if (!btn_ok_prev_is_pressed) {
      btn_ok_pressed_start_time = currentTime;
      btn_ok_prev_is_pressed = true;
    }

    if (currentTime - btn_ok_pressed_start_time >= SWITCH_MODE_DELAY_MS) {
      if (Serial.availableForWrite()) {
        Serial.println("switching mode...");
      }

      /* switch mode */
      priv.mode = priv.mode ^ 1;

      /* update some values because of the new mode */
      if (priv.mode == MODE_RELATIVE) {
        priv.rel_alti_ref = priv.last_alti_m;
      }

      /* start a new count */
      btn_ok_pressed_start_time = currentTime;
    }
  } else {
    btn_ok_prev_is_pressed = false;
  }

  /* SET button */
  if (buttonSet.isPressed()) {
    if (priv.mode == MODE_RELATIVE) {
      priv.rel_alti_ref = priv.last_alti_m;
    } else {
      /* switch to config state */
      return state_conf;
    }
  }

  /* UP + DOWN => use another sensor */
  if (!latchSwitchSnsr && buttonUp.getState() == 0 && buttonDown.getState() == 0) {
    if (!switchSnsrSeqStarted) {
      switch_snsr_seq_start_time = currentTime;
      switchSnsrSeqStarted = true;
    }

    if (currentTime - switch_snsr_seq_start_time >= SWITCH_SNSR_DELAY_MS) {
      /* switch to another sensor if possible */
      latchSwitchSnsr = true;
      switchSnsrSeqStarted = false;

      switch_to_next_snsr();
    }
  } else {
    if (switchSnsrSeqStarted)
      switchSnsrSeqStarted = false;

    if (latchSwitchSnsr)
      latchSwitchSnsr = false;
  }

  return state_acq;
}

static unsigned int sm_conf(void)
{
  static int lastStateBtnUp = 1,
    lastStateBtnDown = 1;
  static size_t cntUp,
    cntDown;
  int btnUpRead,
    btnDownRead;

  priv.dt_ms = DT_FAST_MS;

  /************************/
  /* handle button events */
  if (buttonOk.isPressed()) {
    return state_acq;
  }

  /* up */
  btnUpRead = buttonUp.getState();
  if (btnUpRead == 0) {
    if (lastStateBtnUp == 0) {
      cntUp = min(cntUp + 1, 59);
    } else {
      cntUp = 5;
    }

    priv.sea_pres_ref = floorf(priv.sea_pres_ref) + ((cntUp / 5) * 1.0f);
  }

  lastStateBtnUp = btnUpRead;

  /* down */
  btnDownRead = buttonDown.getState();
  if (btnDownRead == 0) {
    if (lastStateBtnDown == 0) {
      cntDown = min(cntDown + 1, 59);
    } else {
      cntDown = 5;
    }

    priv.sea_pres_ref = floorf(priv.sea_pres_ref) - ((cntDown / 5) * 1.0f);
  }

  lastStateBtnDown = btnDownRead;

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

  delay(priv.dt_ms);
}
