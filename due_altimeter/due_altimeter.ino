/*
* Anthony V.
*
* Dependencies:
* - bme280 library
* - u8g2 library
* - simpleKalmanFilter library
*
* Hardware:
* - Arduino DUE (because the floating number can be double precision
*                but does it make really a difference with simple precision?)
* - OLED Yellow&Blue Display 0.96(SSD1315) connected through I2C
* - BME 280 connected through SPI
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

#include <Wire.h>

/*
 * SimpleKalmanFilter(e_mea, e_est, q);
 * e_mea: Measurement Uncertainty
 * e_est: Estimation Uncertainty
 * q: Process Noise
 */
SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);

/* Grove - OLED Yellow&Blue Display 0.96(SSD1315) */
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);

/* BME-280 on SPI using CS pin 53 */
bfs::Bme280 bme(&SPI, 53);

double getAltitude( double pres, double pres_ref, double temp )
{
  double a1 = pow( ( pres_ref / pres ), ( 1 / 5.257 ) );
  double a = (a1 - 1) * ( temp + 273.15 ) / 0.0065f;
  return a;
}

void oled_display(double alti)
{
  char buf[16];

  snprintf(buf, sizeof(buf), "%.03f", alti);

  u8g2.clearBuffer();                   // clear the internal memory
  u8g2.setFont(u8g2_font_luBIS08_tf);   // choose a suitable font
  u8g2.drawStr(0,10,"Altitude (m)");    // write something to the internal memory
  u8g2.drawStr(0,50,buf);    // write something to the internal memory
  u8g2.sendBuffer();                    // transfer internal memory to the display
}

void setup() {
  /* Serial monitor for showing status and data */
  Serial.begin(115200);
  while (!Serial) {}
  /* Initialize the SPI bus */
  SPI.begin();
  /* Initialize OLED */
  u8g2.begin();
  /* Initialize the BME-280 */
  if (!bme.Begin()) {
    Serial.println("Error initializing communication with BME-280");
    while (1) {}
  }
}

void loop() {
  double pres_pa,
    temp_c,
    hum_rh,
    alti_m;
  float estim_alti_m;

  if (bme.Read()) {
    pres_pa = bme.pres_pa();
    temp_c = bme.die_temp_c();
    hum_rh = bme.humidity_rh();
    alti_m = getAltitude(pres_pa, 101460.0, temp_c);
    estim_alti_m = pressureKalmanFilter.updateEstimate(alti_m);

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
    Serial.print(",");
    Serial.print("estim_alti(m):");
    Serial.println(estim_alti_m);

    oled_display(estim_alti_m);
  }
  delay(100);
}
