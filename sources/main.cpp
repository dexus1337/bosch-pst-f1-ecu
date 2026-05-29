/**
 * @file main.cpp
 * @brief Main entry point for the Bosch PST F1 ECU dashboard display.
 * 
 * This file contains the setup and main loop for reading sensor data
 * (battery voltage, oil pressure, and oil temperature) and displaying 
 * it on a 128x64 SSD1306 OLED screen. It includes the logic for ADC 
 * conversions, thermistor calculations using the Steinhart-Hart equation, 
 * and temporal averaging of the readings for display stability.
 */
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
static constexpr unsigned int    sensor_temperature_pin  = A2;
static constexpr unsigned int    sensor_pressure_pin     = A3;

static constexpr float adc_max                           = 1023.0f;

// Modify these for more exact voltage reading
static constexpr unsigned int    voltage_10k_resistance  = 9836;
static constexpr unsigned int    voltage_2_2k_resistance = 2150;

// Modify these for more exact temperature reading
static constexpr unsigned int    temp_4_6k_resistance    = 4550;

//#define WOWKI_SIM

bool error = false;

/**
 * @brief Initializes the system, display, and pins.
 * 
 * Configures the built-in LED pin, initializes the SSD1306 OLED display,
 * and shows a startup splash screen (AMG logo). Sets an error flag if 
 * the display initialization fails.
 */
void setup(void) 
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

/**
 * @brief Reads and calculates the car's battery voltage.
 * 
 * Samples the analog voltage pin, scales the ADC value to a pin voltage,
 * and applies a voltage divider ratio to determine the actual car 
 * battery voltage.
 * 
 * @return float The calculated car battery voltage in Volts.
 */
float read_car_voltage(void) 
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

/**
 * @brief Reads and calculates the sensor pressure.
 * 
 * Samples the analog pressure pin, scales it to a voltage, and maps
 * that voltage linearly to a pressure value in bar. Clamps negative 
 * pressure values to 0.
 * 
 * @return float The calculated sensor pressure in bar.
 */
float read_sensor_pressure(void) 
{
    int raw_voltage = analogRead(sensor_pressure_pin);
    float pin_voltage = raw_voltage * (5.0f / adc_max);
    
    float pressure_bar = (pin_voltage - 0.5f) * (10.0f / 4.0f);

    if (pressure_bar < 0)
        pressure_bar = 0;

    return pressure_bar;
}

/**
 * @brief Reads and calculates the sensor temperature.
 * 
 * Uses an NTC thermistor to measure temperature. Samples the analog 
 * temperature pin, calculates the current resistance, and applies the 
 * Steinhart-Hart equation to derive the temperature.
 * 
 * @return float The calculated sensor temperature in degrees Celsius.
 */
float read_sensor_temperature(void) 
{
    #if defined WOWKI_SIM
    static constexpr float series_resistor      = 10000.f; 
    static constexpr float nominal_temperature  = 25.0f;
    static constexpr float nominal_resistance   = 10000.0f;
    static constexpr float beta_coefficient     = 3020.0f;
    #else
    static constexpr float nominal_temperature  = 20.0f;
    static constexpr float nominal_resistance   = 2480.0f;
    static constexpr float beta_coefficient     = 3480.0f;
    #endif

    // based on steinhart equation for NTC thermistors
    int raw_adc = analogRead(sensor_temperature_pin);

    #if defined WOWKI_SIM
    float resistance = series_resistor / ((adc_max / (float)raw_adc) - 1.0f);
    #else
    float resistance = temp_4_6k_resistance / ((adc_max / (float)raw_adc) - 1.0f);
    #endif

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

/**
 * @brief Main application loop.
 * 
 * Handles the built-in LED error state blinking, continuously reads
 * sensor data (voltage, pressure, temperature), and averages every 
 * 10 readings. The averaged results are then pushed to the OLED display
 * alongside the corresponding icons.
 */
void loop(void) 
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