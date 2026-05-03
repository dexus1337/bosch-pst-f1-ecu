#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/Org_01.h>

#include "../resources/amg-logo.hpp"
#include "../resources/battery-logo.hpp"
#include "../resources/oil-logo.hpp"

static constexpr int screen_width = 128;
static constexpr int screen_height = 64;

Adafruit_SSD1306 
display
(
    screen_width, 
    screen_height, 
    &Wire, 
    -1 
);

static constexpr unsigned int    voltage_pin             = A1;
static constexpr unsigned int    sensor_temperature_pin  = A3;
static constexpr unsigned int    sensor_pressure_pin     = A2;

static constexpr float adc_max                           = 1023.0f;

// Modify these for more exact voltage reading
static constexpr unsigned int    voltage_10k_resistance  = 9980;
static constexpr unsigned int    voltage_2_2k_resistance = 2150;

// Modify these for more exact temperature reading
static constexpr unsigned int    temp_4_6k_resistance    = 4550;

//#define WOWKI_SIM

bool error = false;

void 
setup
(
    void
) 
{
    delay(1000);

    // We use the built-in pin to signalize any kind of error
    pinMode(LED_BUILTIN, OUTPUT);

    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) 
        error = true;

    display.clearDisplay();

    display.drawBitmap(0, 15, amg_logo, amg_logo_width, amg_logo_height, WHITE);

    display.setFont(&Org_01);
    display.setTextColor(WHITE);
    display.setCursor(42, 50);
    display.println("Starting...");

    display.display();

    delay(2000);
}

float 
read_car_voltage
(
    void
) 
{
    int raw_voltage = analogRead(voltage_pin);
    
    // 1. Convert ADC steps to actual voltage at the pin
    float pin_voltage = raw_voltage * (5.0f / adc_max);
    
    // 2. Calculate the divider ratio dynamically
    // Ratio = (R1 + R2) / R2
    float divider_ratio = (static_cast<float>(voltage_10k_resistance) + static_cast<float>(voltage_2_2k_resistance)) / static_cast<float>(voltage_2_2k_resistance);
    
    // 3. Scale up to find the actual car battery voltage
    float car_voltage = pin_voltage * divider_ratio; 
    
    return car_voltage;
}

float 
read_sensor_pressure
(
    void
) 
{
    int raw_voltage = analogRead(sensor_pressure_pin);
    float pin_voltage = raw_voltage * (5.0f / adc_max);
    
    float pressure_bar = (pin_voltage - 0.5f) * (10.0f / 4.0f);

    if (pressure_bar < 0)
        pressure_bar = 0;

    return pressure_bar;
}

float 
read_sensor_temperature
(
    void
) 
{
    #if defined WOWKI_SIM
    static constexpr float series_resistor      = 10000.f; 
    static constexpr float nominal_temperature  = 25.0f;
    static constexpr float nominal_resistance   = 10000.0f;
    static constexpr float beta_coefficient     = 3020.0f;
    #else
    static constexpr float nominal_temperature  = 20.0f;
    static constexpr float nominal_resistance   = 2480.0f;  // resistance at nominal temp
    static constexpr float beta_coefficient     = 3480.0f;  // calculated, maybe switch to linear calibration later, we have all the values
    
    #endif

    // based on steinhart equation for NTC thermistors
    int raw_adc = analogRead(sensor_temperature_pin);

    float resistance = temp_4_6k_resistance / ((adc_max / (float)raw_adc) - 1.0f);

    float temperature_kelvin;
    temperature_kelvin = log(resistance / nominal_resistance);      // ln(R/Ro)
    temperature_kelvin /= beta_coefficient;                         // 1/B * ln(R/Ro)
    temperature_kelvin += 1.0f / (nominal_temperature + 273.15f);   // + (1/To)
    temperature_kelvin = 1.0f / temperature_kelvin;                 // invert

    return temperature_kelvin - 273.15f;
}

float voltage_readings[10];
float pressure_readings[10];
float temperature_readings[10];

int cur_index       = 0;
unsigned long time  = 0;
bool led_state      = false;

void 
loop
(
    void
) 
{
    auto cur_time = millis();

    if (!error)
    {
        digitalWrite(LED_BUILTIN, HIGH);
    }
    else
    {
        if ( cur_time - time > 1000ul)
        {
            led_state   = !led_state;
            time        = cur_time;

            digitalWrite(LED_BUILTIN, led_state);
        }
    }
    
    voltage_readings[cur_index]       = read_car_voltage();
    pressure_readings[cur_index]      = read_sensor_pressure();
    temperature_readings[cur_index]   = read_sensor_temperature();

    cur_index++;

    if (cur_index >= 10)
        cur_index = 0;
    
    if ( cur_index != 0 )
        return;

    float average_voltage = 0;
    float average_pressure = 0;
    float average_temperature = 0;

    for (int i = 0; i < 10; i++)
    {
        average_voltage += voltage_readings[i];
        average_pressure += pressure_readings[i];
        average_temperature += temperature_readings[i];
    }

    average_voltage /= 10.f;
    average_pressure /= 10.f;
    average_temperature /= 10.f;

    display.clearDisplay();

    display.drawBitmap(20, 17, battery_logo_small, battery_logo_small_width, battery_logo_small_height, WHITE);
    display.drawBitmap(75, 17, oil_logo_small, oil_logo_small_width, oil_logo_small_height, WHITE); 

    display.setFont(&Org_01);
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(7, 47);
    display.println(String(average_voltage, 1) + "V");

    display.setCursor(65, 47);
    display.println(String(average_pressure, 1) + "bar");
    display.setCursor(65, 60);
    display.println(String(average_temperature, 1) + "°C");
    
    display.display();
}