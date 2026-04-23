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

static constexpr int    voltage_pin             = A1;
static constexpr int    sensor_pressure_pin     = A2;
static constexpr int    sensor_temperature_pin  = A3;
static constexpr float  adc_max                 = 1023.0f;

#define WOWKI_SIM

void 
setup
(
    void
) 
{
    Serial.begin( 115200 );

    if( !display.begin( SSD1306_SWITCHCAPVCC, 0x3C ) ) 
    {
        Serial.println( F("OLED konnte nicht gefunden werden") );
        return;
    }

    display.clearDisplay( );

    display.drawBitmap( 0, 15, amg_logo, amg_logo_width, amg_logo_height, WHITE );

    display.setFont( &Org_01 );
    display.setTextColor( WHITE );
    display.setCursor( 42, 50 );
    display.println( "Starting..." );

    display.display( );

    delay( 2000 );
}

float 
read_car_voltage
(
    void
) 
{
    int raw_voltage = analogRead( voltage_pin );
    float pin_voltage = raw_voltage * ( 5.0f / adc_max );
    
    // Reverse the divider: R1=10k, R2=2.2k -> Ratio is (10+2.2)/2.2 = 5.545
    float car_voltage = pin_voltage * 5.5454f; 
    return car_voltage;
}

float 
read_sensor_pressure
(
    void
) 
{
    int raw_voltage = analogRead( sensor_pressure_pin );
    float pin_voltage = raw_voltage * ( 5.0f / adc_max );
    
    float pressure_bar = ( pin_voltage - 0.5f ) * ( 10.0f / 4.0f );

    if ( pressure_bar < 0 )
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
    static constexpr float series_resistor      = 4600.f;   // bosch
    static constexpr float nominal_temperature  = 20.0f;
    static constexpr float nominal_resistance   = 2480.0f;  // resistance at nominal temp
    static constexpr float beta_coefficient     = 3480.0f;  // calculated, maybe switch to linear calibration later, we have all the values
    
    #endif

    // based on steinhart equation for NTC thermistors
    int raw_adc = analogRead( sensor_temperature_pin );

    float resistance = series_resistor / ((adc_max / (float)raw_adc) - 1.0f);

    float temperature_kelvin;
    temperature_kelvin = log(resistance / nominal_resistance);      // ln(R/Ro)
    temperature_kelvin /= beta_coefficient;                         // 1/B * ln(R/Ro)
    temperature_kelvin += 1.0f / (nominal_temperature + 273.15f);   // + (1/To)
    temperature_kelvin = 1.0f / temperature_kelvin;                 // invert

    return temperature_kelvin - 273.15f;
}

void 
loop
(
    void
) 
{
    display.clearDisplay( );

    display.drawBitmap( 20, 10, battery_logo_small, battery_logo_small_width, battery_logo_small_height, WHITE );

    display.drawBitmap( 75, 10, oil_logo_small, oil_logo_small_width, oil_logo_small_height, WHITE ); 

    float voltage       = read_car_voltage( );
    float pressure      = read_sensor_pressure( );
    float temperature   = read_sensor_temperature( );

    display.setFont( &Org_01 );
    display.setTextSize( 2 );
    display.setTextColor( WHITE );
    display.setCursor( 7, 47 );
    display.println( String( voltage, 1 ) + "V" );
    
    display.setCursor( 65, 47 );
    display.println( String( pressure, 1 ) + "bar" );
    display.setCursor( 65, 60 );
    display.println( String( temperature, 1 ) + "°C" );
    
    display.display( );

    delay( 100 );
}