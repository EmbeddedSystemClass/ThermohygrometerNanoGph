#include <math.h>
#include <EEPROM.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <NeoPixelPainter.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1008)

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

#define SECONDS                 ((uint8_t) 2)

#define ARRAY_LEN               100
#define ARRAY_START             1

#define SCREEN_WIDTH            128 // OLED display width, in pixels
#define SCREEN_HEIGHT           64 // OLED display height, in pixels

#define GRAPH_X                 (SCREEN_WIDTH - (ARRAY_LEN + 1))
#define GRAPH_Y                 16
#define GRAPH_W                 (ARRAY_LEN + 1)
#define GRAPH_H                 47
#define DATA_X                  10
#define DATA_Y                  0

#define DSP_AWAKE               10
#define LED_AWAKE               5

volatile uint8_t seconds_counter = 0;
volatile uint16_t seconds_counter_state = 0;
volatile uint32_t seconds_counter_sample = 0;

short i_stamp = 0;

uint8_t minutes = 5;

volatile uint8_t state = STATE_IDLE;

float temperature = -1, humidity = -1;
float pressure = -1, altitude = -1;
float t1 = -1, h1 = -1;
float t2 = -1, h2 = -1;
float t_max = -1, t_min = -1, h_max = -1, h_min = -1;

bool first = true;

uint8_t click = 0;
bool long_press_mode = false;
bool enter_long_press_mode = false;
bool was_long_press_mode = false;
bool long_click = 0;
bool last_click_state = false;
long last_click_state_timestamp = -1;
long click_state_timestamp = -1;

volatile bool busy = false;
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

  for (short i = 0; i < i_stamp; i++) {
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

  for (short i = 0; i < i_stamp; i++) {
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

void readInput() {
  int val = digitalRead(INPUT_CMD);
  if (val == HIGH) {
    last_click_state_timestamp = millis();
    if (click_state_timestamp < 0) {
      click_state_timestamp = last_click_state_timestamp;
    }
    if ((last_click_state_timestamp - click_state_timestamp) > 3000 && !was_long_press_mode) {
      state = STATE_SETTINGS;

      if (long_press_mode) {
        long_click = true;
      }

      enter_long_press_mode = true;
      long_press_mode = true;
      was_long_press_mode = true;
    }
    last_click_state = HIGH;
  } else {
    long timestamp = millis();
    if (last_click_state == HIGH && (timestamp - last_click_state_timestamp) > 100) {
      if (enter_long_press_mode) {
        enter_long_press_mode = false;
      } else if (long_press_mode) {
        long_press_mode = false;
        state = STATE_WAKE_LED;
      } else {
        click++;
      }

      refreshDisplayFlag = true;

      was_long_press_mode = false;
      click_state_timestamp = -1;
      last_click_state_timestamp = timestamp;
      seconds_counter_state = 0;
      last_click_state = LOW;
    }
  }
}

void displaySensors() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

//  display.println(F("SENSORS:"));
//  display.println(F("AM2302 & BME280"));

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

      if (long_click) {
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
        long_click = false;
      }

      EEPROM.write(0, (byte) minutes);

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
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
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

//void printValues() {
//  bme.takeForcedMeasurement();
//
//  Serial.print(F("Temperature = "));
//  Serial.print(bme.readTemperature());
//  Serial.println(F(" *C"));
//
//  Serial.print(F("Pressure = "));
//
//  Serial.print(bme.readPressure() / 100.0F);
//  Serial.println(F(" hPa"));
//
//  Serial.print(F("Approx. Altitude = "));
//  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
//  Serial.println(F(" m"));
//
//  Serial.print(F("Humidity = "));
//  Serial.print(bme.readHumidity());
//  Serial.println(F(" %"));
//
//  Serial.println();
//}

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
    Serial.println(F("SSD1306 allocation failed"));
    delay(500);
  }
  delay(500);
  Serial.println(F("*>DSPINIT"));

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

  Serial.print(F("*>MINUTES:"));
  Serial.println(minutes);

  i2cScanner();

  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  if (! bme.begin()) {
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
  }

  // weather monitoring
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF);

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

    float _h = (float) dht.readHumidity();
    float _t = (float) dht.readTemperature();

    bme.takeForcedMeasurement();

    float _t1 = (float) bme.readTemperature();
    float _h1 = (float) bme.readHumidity();

    float _p = (float) bme.readPressure() / 100.0F;
    float _a = (float) bme.readAltitude(SEALEVELPRESSURE_HPA);

    if (!isnan(_h) && !isnan(_t)) {
      h1 = (_h + H_OFFSET1);
      t1 = (_t + T_OFFSET1);
    }

    if (!isnan(_h1) && !isnan(_t1)) {
      h2 = (_h1 + H_OFFSET2);
      t2 = (_t1 + T_OFFSET2);
    }

    if (!isnan(_p) && !isnan(_a)) {
      pressure = _p;
      altitude = _a;
    }

    if (!isnan(h1) && !isnan(t1) && !isnan(h2) && !isnan(t2)) {
      humidity = (h1 + h2) / 2.;
      temperature = (t1 + t2) / 2.;

      Serial.print(F("*>T:"));
      Serial.println(temperature);
      Serial.print(F("*>H:"));
      Serial.println(humidity);
      Serial.flush();


      Serial.print(F("*>P:"));
      Serial.println(pressure);
      Serial.print(F("*>A:"));
      Serial.println(altitude);
      Serial.flush();

      Serial.print(F("*>T1:"));
      Serial.println(t1);
      Serial.print(F("*>H1:"));
      Serial.println(h1);
      Serial.flush();

      Serial.print(F("*>T2:"));
      Serial.println(t2);
      Serial.print(F("*>H2:"));
      Serial.println(h2);
      Serial.flush();

      if (first) {
        t_min = temperature;
        t_max = temperature;
        h_max = humidity;
        h_min = humidity;
      }

      readInput();

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

  readInput();

  // Check state for FSM
  checkState();

  readInput();

  // Write data on EEPROM
  writeEEPROM();

  readInput();
}
