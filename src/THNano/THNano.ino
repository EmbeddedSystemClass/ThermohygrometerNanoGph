#include <math.h>
#include <EEPROM.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <NeoPixelPainter.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA    1000
#define PRESSURE_HPA_MIN        980
#define PRESSURE_HPA_MAX        1045

#define LED_PIN                 2
#define LED_COUNT               1

#define INPUT_CMD               4

#define DHT_PIN                 3
#define DHTTYPE                 DHT22

#define T_OFFSET1               0.0
#define H_OFFSET1               0.0

#define T_OFFSET2               0.0
#define H_OFFSET2               0.0

#define STATE_IDLE              0
#define STATE_INIT              1
#define STATE_SLEEP             2
#define STATE_WAKE_LED          3
#define STATE_WAKE              4
#define STATE_TEMPERATURE       5
#define STATE_HUMIDITY          6
#define STATE_SENSORS           7
#define STATE_SETTINGS          8
#define STATE_CLEAR             9
#define STATE_SET_HPA           10

#define SECONDS                 ((uint8_t) 2)

#define ARRAY_LEN               100
#define ARRAY_START             4

#define SCREEN_WIDTH            128 // OLED display width, in pixels
#define SCREEN_HEIGHT           64 // OLED display height, in pixels

#define GRAPH_X                 (SCREEN_WIDTH - (ARRAY_LEN + 1))
#define GRAPH_Y                 16
#define GRAPH_W                 (ARRAY_LEN + 1)
#define GRAPH_H                 47
#define DATA_X                  10
#define DATA_Y                  0

#define DSP_AWAKE               20
#define LED_AWAKE               10

volatile uint8_t seconds_counter = 0;
volatile uint16_t seconds_counter_state = 0;
volatile uint32_t seconds_counter_sample = 0;

uint8_t i_stamp = 0;

uint16_t sea_pressure = SEALEVELPRESSURE_HPA;

uint8_t minutes = 5;

volatile uint8_t state = STATE_IDLE;

float temperature = ((float) 0xFFFFFFFF), humidity = ((float) 0xFFFFFFFF);
float pressure = ((float) 0xFFFFFFFF), altitude = ((float) 0xFFFFFFFF);
float t1 = ((float) 0xFFFFFFFF), h1 = ((float) 0xFFFFFFFF);
float t2 = ((float) 0xFFFFFFFF), h2 = ((float) 0xFFFFFFFF);
float t_max = ((float) 0xFFFFFFFF), t_min = ((float) 0xFFFFFFFF), h_max = ((float) 0xFFFFFFFF), h_min = ((float) 0xFFFFFFFF);

bool first = true;

uint8_t click = 0;
uint8_t long_click = 0;

bool last_click_state = LOW;
long start_press_timestamp = -1;
long press_timestamp = -1;

bool displayWake = true;

bool refreshDisplayFlag = false;

// Create an instance of the Adafruit_NeoPixel class called "leds".
// That'll be what we refer to from here on...
Adafruit_NeoPixel leds = Adafruit_NeoPixel(LED_COUNT, LED_PIN, NEO_RGB + NEO_KHZ800);

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// DHT instance
DHT dht(DHT_PIN, DHTTYPE);

Adafruit_BME280 bme;

void sleepDisplay() {
  if (displayWake) {
    display.ssd1306_command(SSD1306_DISPLAYOFF);
    displayWake = false;
  }
}

void wakeDisplay() {
  if (!displayWake) {
    display.ssd1306_command(SSD1306_DISPLAYON);
    displayWake = true;
  }
}

void refreshLed() {
  if (temperature >= 25 || humidity >= 75) {
    leds.setPixelColor(0, 0x4F0000);
    leds.show();
  } else if (temperature >= 18 || humidity >= 60) {
    leds.setPixelColor(0, 0x4F4F00);
    leds.show();
  } else if (temperature >= 18 || humidity >= 50) {
    leds.setPixelColor(0, 0x0A4F00);
    leds.show();
  } else {
    leds.setPixelColor(0, 0x004F00);
    leds.show();
  }
}

ISR(TIMER1_COMPA_vect) {
  if (state > STATE_INIT) {
    seconds_counter++;
  }
  seconds_counter_sample++;
  seconds_counter_state++;
}

void setLEDColor(const long color) {
  leds.setPixelColor(0, color);
  leds.show();
}

void clearLEDs() {
  leds.setPixelColor(0, 0x0);
  leds.show();
}

void displayGraph(const byte displayMode) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  if (displayMode == 0)
    display.print(F("TEMPERATURE:"));
  else if (displayMode == 1)
    display.print(F("HUMIDITY:"));

  display.drawFastVLine(GRAPH_X, GRAPH_Y, GRAPH_H, WHITE);
  display.drawFastHLine(GRAPH_X, GRAPH_H + GRAPH_Y, GRAPH_W, WHITE);

  short offset = ARRAY_LEN * displayMode;

  //int mean = 0;
  byte mean = 0;
  byte max = (byte) EEPROM.read(ARRAY_START + offset);
  byte min = (byte) EEPROM.read(ARRAY_START + offset);

  for (uint8_t i = 0; i < i_stamp; i++) {
    //mean += (byte) EEPROM.read(ARRAY_START + i + offset);
    if (max < ((byte) EEPROM.read(ARRAY_START + i + offset))) {
      max = (byte) EEPROM.read(ARRAY_START + i + offset);
    }
    if (min > ((byte) EEPROM.read(ARRAY_START + i + offset))) {
      min = (byte) EEPROM.read(ARRAY_START + i + offset);
    }
  }
  //mean = (int) (mean / i_stamp);
  mean = ((max + min) / 2);
  byte max_delta = (byte) max((byte) (abs(mean - max)), (byte) (abs(mean - min)));

  byte x = 0;
  x = (byte) ((GRAPH_Y + (GRAPH_H / 2)) + (((GRAPH_H / 2) * (- max + mean)) / max_delta));
  display.drawPixel(GRAPH_X - 1, x, WHITE);
  display.setCursor(0,  x);
  display.print(max);
  if (displayMode == 0)
    display.print((char)247);
  else if (displayMode == 1)
    display.print(F("%"));

  x = (byte) ((GRAPH_Y + (GRAPH_H / 2)) + (((GRAPH_H / 2) * (- min + mean)) / max_delta));
  display.drawPixel(GRAPH_X - 1, x, WHITE);
  display.setCursor(0, x - 6);
  display.print(min);
  if (displayMode == 0)
    display.print((char)247);
  else if (displayMode == 1)
    display.print(F("%"));

  display.drawPixel(GRAPH_X - 1, GRAPH_Y + (GRAPH_H / 2), WHITE);
  display.setCursor(0,  GRAPH_Y + (GRAPH_H / 2) - 3);
  display.print(mean);
  if (displayMode == 0)
    display.print((char)247);
  else if (displayMode == 1)
    display.print(F("%"));

  for (uint8_t i = 0; i < i_stamp; i++) {
    byte _t = (byte) EEPROM.read(ARRAY_START + i + offset);
    x = (byte) ((GRAPH_Y + (GRAPH_H / 2)) + (((GRAPH_H / 2) * (- _t + mean)) / max_delta));
    display.drawPixel(GRAPH_X + i + 1, x, WHITE);
  }

  display.display();
}

void refreshDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  display.print(F("T:  "));
  display.print(temperature);
  display.print(F(" "));
  display.print((char)247);
  display.println(F("C"));

  display.print(F("RH: "));
  display.print(humidity);
  display.println(F(" %"));

  display.print(F("T Max: "));
  display.println(t_max);
  display.print(F("T Min: "));
  display.println(t_min);
  display.print(F("RH Max: "));
  display.println(h_max);
  display.print(F("RH Min: "));
  display.println(h_min);

  display.println();
  display.print(F("Samples: "));
  display.print(i_stamp);
  display.print(F(". Rate: "));
  display.println(minutes);

  display.display();
}

void readClick() {
  bool val = digitalRead(INPUT_CMD);
  if (val == HIGH) {
    if (last_click_state == LOW) {
      start_press_timestamp = millis();
    }
    last_click_state = HIGH;
    press_timestamp = millis();
  } else {
    long timestamp = millis();
    if ((last_click_state == HIGH) && ((abs(timestamp - press_timestamp)) >= 50)) {
      if (abs(press_timestamp - start_press_timestamp) > 2000) {
        ++long_click;
      } else {
        ++click;
      }
      last_click_state = LOW;
      refreshDisplayFlag = true;
      seconds_counter_state = 0;
      press_timestamp = -1;
      start_press_timestamp = -1;
    }
  }
}

void displaySensors() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  display.println(F("AM2302:"));

  display.print(F("T:  "));
  display.print(t1);
  display.print(F(" "));
  display.print((char)247);
  display.println(F("C"));

  display.print(F("RH: "));
  display.print(h1);
  display.println(F(" %"));

  display.println(F("BME280:"));

  display.print(F("T:  "));
  display.print(t2);
  display.print(F(" "));
  display.print((char)247);
  display.println(F("C"));

  display.print(F("RH: "));
  display.print(h2);
  display.println(F(" %"));

  display.print(F("Pres = "));
  display.print(pressure);
  display.println(F(" hPa"));

  display.print(F("Alt = "));
  display.print(altitude);
  display.println(F(" m"));

  display.display();
}

void checkState() {
  switch (state) {
    case STATE_SLEEP:
      clearLEDs();
      leds.show();

      sleepDisplay();

      if (click > 0) {
        --click;
        state = STATE_WAKE_LED;
      }
      if (long_click > 0) {
        --long_click;
        state = STATE_SETTINGS;
      }

      break;
    case STATE_WAKE_LED:
      wakeDisplay();

      refreshLed();
      if (refreshDisplayFlag) {
        refreshDisplay();
        display.display();
        refreshDisplayFlag = false;
      }

      if (seconds_counter_state > LED_AWAKE) {
        state = STATE_WAKE;
      }

      if (click > 0) {
        --click;
        state = STATE_TEMPERATURE;
      }
      if (long_click > 0) {
        --long_click;
        state = STATE_SETTINGS;
      }

      break;
    case STATE_WAKE:
      clearLEDs();
      leds.show();

      wakeDisplay();

      if (refreshDisplayFlag) {
        refreshDisplay();
        display.display();
        refreshDisplayFlag = false;
      }

      if (seconds_counter_state > DSP_AWAKE) {
        state = STATE_SLEEP;
      }

      if (click > 0) {
        --click;
        state = STATE_WAKE_LED;
      }
      if (long_click > 0) {
        --long_click;
        state = STATE_SETTINGS;
      }

      break;
    case STATE_TEMPERATURE:
      if (refreshDisplayFlag) {
        displayGraph(0);
        display.display();
        refreshDisplayFlag = false;
      }

      if (seconds_counter_state > DSP_AWAKE) {
        state = STATE_SLEEP;
      }


      if (click > 0) {
        --click;
        state = STATE_HUMIDITY;
      }
      if (long_click > 0) {
        --long_click;
        state = STATE_SETTINGS;
      }

      break;
    case STATE_HUMIDITY:
      if (refreshDisplayFlag) {
        displayGraph(1);
        display.display();
        refreshDisplayFlag = false;
      }

      if (seconds_counter_state > DSP_AWAKE) {
        state = STATE_SLEEP;
      }

      if (click > 0) {
        --click;
        state = STATE_SENSORS;
      }
      if (long_click > 0) {
        --long_click;
        state = STATE_SETTINGS;
      }

      break;
    case STATE_SENSORS:
      if (refreshDisplayFlag) {
        displaySensors();
        display.display();
        refreshDisplayFlag = false;
      }

      if (seconds_counter_state > DSP_AWAKE) {
        state = STATE_SLEEP;
      }

      if (click > 0) {
        --click;
        state = STATE_WAKE_LED;
      }
      if (long_click > 0) {
        --long_click;
        state = STATE_SETTINGS;
      }

      break;
    case STATE_SETTINGS:
      wakeDisplay();
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0, 0);
      display.println(F("### SETTINGS ###"));
      display.println();
      display.println(F("SAPLES RATE:"));
      display.print(minutes);
      display.println(F(" minutes"));
      display.display();

      if (click > 0) {
        switch (minutes) {
          case 5:
            minutes = 10;
            break;
          case 10:
            minutes = 1;
            break;
          default:
            minutes = 5;
            break;
        }
        EEPROM.write(0, (byte) minutes);
        --click;
      }
      if (long_click > 0) {
        --long_click;
        state = STATE_CLEAR;
      }
      seconds_counter_state = 0;

      break;
    case STATE_CLEAR:
      wakeDisplay();
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0, 0);
      display.println(F("### CLEAR SAMPLES ###"));
      display.println();
      display.println(F("Click to skip"));
      display.println(F("Long click to clear"));
      display.display();

      if (long_click > 0) {
        --long_click;
        state = STATE_SET_HPA;

        EEPROM.write(1, (byte) 0x00);
        i_stamp = 0;
      }

      if (click > 0) {
        --click;
        state = STATE_SET_HPA;
      }
      seconds_counter_state = 0;
      
      break;
    case STATE_SET_HPA:
      wakeDisplay();
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0, 0);
      display.println(F("### SET PRESSURE ###"));
      display.println();
      display.println(F("Sea level pressure:"));
      display.print(sea_pressure);
      display.println(F(" hPa"));
      display.println(F("Click to increase"));
      display.println(F("Long click to save"));
      display.display();

      if (click > 0) {
        --click;
        ++sea_pressure;
        if(sea_pressure > PRESSURE_HPA_MAX) {
          sea_pressure = PRESSURE_HPA_MIN;
        }
      }
      if (long_click > 0) {
        --long_click;
        state = STATE_WAKE_LED;

        EEPROM.write(2, (byte) ((sea_pressure & 0xFF00) >> 8));
        EEPROM.write(3, (byte) (sea_pressure & 0x00FF));
      }
      
      break;
  }

  if (seconds_counter_state > (DSP_AWAKE + 1)) {
    seconds_counter_state = 0;
  }
}

void i2cScanner() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ ) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print(F("I2C device found at address 0x"));
      if (address < 16)
        Serial.print(F("0"));
      Serial.print(address, HEX);
      Serial.println(F("!"));

      nDevices++;
    } else if (error == 4) {
      Serial.print(F("Error at address 0x"));
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println(F("No I2C device"));
  else
    Serial.println(F("done"));
}

void writeEEPROM() {
  if (seconds_counter_sample >= (minutes * 60L)) {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.print("*>SAMPLE:");
    Serial.println(seconds_counter_sample);

    seconds_counter_sample = 0;
    if (i_stamp < ARRAY_LEN) {
      EEPROM.write(ARRAY_START + i_stamp, (byte) round(temperature));
      EEPROM.write(ARRAY_START + i_stamp + ARRAY_LEN, (byte) round(humidity));
      i_stamp++;
      EEPROM.write(1, (byte) i_stamp);
    } else {
      for (short i = 0; i < ARRAY_LEN - 1; i++) {
        EEPROM.write(ARRAY_START + i, EEPROM.read(ARRAY_START + i + 1));
        EEPROM.write(ARRAY_START + i + ARRAY_LEN, EEPROM.read(ARRAY_START + i + 1 + ARRAY_LEN));
      }
      EEPROM.write(ARRAY_START + ARRAY_LEN - 1, (byte) temperature);
      EEPROM.write(ARRAY_START + ARRAY_LEN - 1 + ARRAY_LEN, (byte) humidity);
    }
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void bmeBegin() {
  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  if (! bme.begin()) {
    Serial.println(F("BME280 BEGIN ERROR!"));
  }

  // weather monitoring
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  first = true;
  state = STATE_INIT;

  Wire.begin();
  Serial.begin(115200);

  pinMode(INPUT_CMD, INPUT);

  leds.begin();  // Call this to start up the LED strip.
  clearLEDs();   // This function, defined below, turns all LEDs off...
  leds.show();   // ...but the LEDs don't actually update until you call this.

  setLEDColor(0xFF0000);
  Serial.println(F("*>INIT"));

  while (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 ERROR"));
    delay(500);
  }
  delay(500);
  Serial.println(F("DSPINIT"));

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println(F("LOADING..."));
  display.println(F("BOOTING ATMega328P..."));
  display.println(F("#####################"));
  display.println(F("T/H"));
  display.println(F("by Niccolo' Ferrari"));
  display.println(F("@madnick_93"));
  display.println(F("CRIME FOR CLIMB"));
  display.println(F("OMEGASOFTWARE"));
  display.println(F("v1.1"));
  display.display();

  cli();
  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei(); //allow interrupts

  // Init DHT
  dht.begin();

  clearLEDs();   // This function, defined below, turns all LEDs off...
  leds.show();   // ...but the LEDs don't actually update until you call this.

  minutes = (byte) EEPROM.read(0);
  if (minutes != 1 && minutes != 5 && minutes != 10) {
    minutes = 5;
    EEPROM.write(0, (byte) minutes);
  }

  Serial.print(F("MINUTES:"));
  Serial.println(minutes);

  i2cScanner();

  bmeBegin();

  byte i_temp = EEPROM.read(1);
  Serial.print(F("SAMPLES1:"));
  Serial.println(i_temp);
  if (i_temp >= 0 && i_temp <= ARRAY_LEN) {
    i_stamp = (uint8_t) i_temp;
  } else {
    i_stamp = 0;
  }
  Serial.print(F("SAMPLES2:"));
  Serial.println(i_stamp);

  byte _h = EEPROM.read(2);
  byte _l = EEPROM.read(3);
  sea_pressure = (uint16_t)((_h << 8) | _l);
  Serial.print(F("PRESSURE1:"));
  Serial.println(sea_pressure);
  if(sea_pressure > PRESSURE_HPA_MAX || sea_pressure < PRESSURE_HPA_MIN) {
    sea_pressure = SEALEVELPRESSURE_HPA;

    EEPROM.write(2, (byte) ((sea_pressure & 0xFF00) >> 8));
    EEPROM.write(3, (byte) (sea_pressure & 0x00FF));
  }
  Serial.print(F("PRESSURE2:"));
  Serial.println(sea_pressure);

  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  if (state == STATE_INIT) {
    state = STATE_WAKE_LED;
    seconds_counter_state = 0;
  }

  if (seconds_counter >= SECONDS) {
    seconds_counter = 0;

    //    printValues();

    uint8_t readVals = 0x00;

    float _h = (float) dht.readHumidity();
    float _t = (float) dht.readTemperature();

    bme.takeForcedMeasurement();

    float _t1 = (float) bme.readTemperature();
    float _h1 = (float) bme.readHumidity();

    float _p = (float) bme.readPressure() / 100.0F;
    float _a = (float) bme.readAltitude(sea_pressure);

    if (!isnan(_t)) {
      readVals |= (0x01 << 0);
      t1 = (_t + T_OFFSET1);
    }
    if (!isnan(_h)) {
      readVals |= (0x01 << 2);
      h1 = (_h + H_OFFSET1);
    }
    if (!isnan(_t1)) {
      readVals |= (0x01 << 1);
      t2 = (_t1 + T_OFFSET2);
    }
    if (!isnan(_h1)) {
      readVals |= (0x01 << 3);
      h2 = (_h1 + H_OFFSET2);
    }
    if (!isnan(_p)) {
      readVals |= (0x01 << 4);
      pressure = _p;
    }
    if (!isnan(_a)) {
      readVals |= (0x01 << 5);
      altitude = _a;
    }

    uint8_t _f = 0x00;
    if ((_f = (readVals & 0x01) + ((readVals >> 1) & 0x01)) > 0) {
      Serial.print(F("TEMPERATURE READS: "));
      Serial.println((int) _f);
      temperature = 0.F;
      if (readVals & 0x01) {
        Serial.print(F("TEMPERATURE FROM AM2302: "));
        Serial.println(t1);
        temperature += t1;
      }
      if (readVals & (0x01 << 1)) {
        Serial.print(F("TEMPERATURE FROM BME280: "));
        Serial.println(t2);
        temperature += t2;
      } else {
        Serial.print(F("BME REINIT: "));
        Serial.println(_t1);
        bmeBegin();
      }
      temperature /= float(_f);
      Serial.print(F("TEMPERATURE MEAN: "));
      Serial.println(temperature);
    }

    _f = 0x00;
    if ((_f = ((readVals >> 2) & 0x01) + ((readVals >> 3) & 0x01)) > 0) {
      Serial.print(F("HUMIDITY READS: "));
      Serial.println((int) _f);
      humidity = 0.F;
      if (readVals & (0x01 << 2)) {
        Serial.print(F("HUMIDITY FROM AM2302: "));
        Serial.println(h1);
        humidity += h1;
      }
      if (readVals & (0x01 << 3)) {
        Serial.print(F("HUMIDITY FROM BME280: "));
        Serial.println(h2);
        humidity += h2;
      } else {
        Serial.print(F("BME REINIT:"));
        Serial.println(_h1);
        bmeBegin();
      }
      humidity /= float(_f);
      Serial.print(F("HUMIDITY MEAN: "));
      Serial.println(humidity);
    }

    Serial.print(F("TEMPERATURE: "));
    Serial.println(temperature);
    Serial.print(F("HUMIDITY: "));
    Serial.println(humidity);
    Serial.print(F("PRESSURE: "));
    Serial.println(pressure);
    Serial.print(F("ALTITUDE: "));
    Serial.println(altitude);
    Serial.println();

    _f = 0x00;
    if (!isnan(humidity) && !isnan(temperature)) {
      if (first) {
        t_min = temperature;
        t_max = temperature;
        h_max = humidity;
        h_min = humidity;
      }

      readClick();

      // Check max and min
      if (temperature > t_max) {
        t_max = temperature;
      }
      if (humidity > h_max) {
        h_max = humidity;
      }
      if (temperature < t_min) {
        t_min = temperature;
      }
      if (humidity < h_min) {
        h_min = humidity;
      }

      refreshDisplayFlag = true;
    }

    if (first) {
      seconds_counter_sample = (minutes * 60L);
    }

    if (first) {
      first = false;
    }
  }

  readClick();

  // Check state for FSM
  checkState();

  readClick();

  // Write data on EEPROM
  writeEEPROM();

  readClick();
}
