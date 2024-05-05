# Import libraries
import I2C_LCD_driver
from time import sleep
import board
import adafruit_dht
from smbus import SMBus
import RPi.GPIO as GPIO
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)

# Define constants
LDR_PIN = 27
DHT11_PIN = board.D17
RELAY_PIN = 17  # Replace with the actual GPIO pin connected to the relay

def main():
    # Initialize GPIO and I2C bus
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    bus = SMBus(1)

    # Set up LDR pin as input
    GPIO.setup(LDR_PIN, GPIO.IN)

    # Set up relay pin as output
    GPIO.setup(RELAY_PIN, GPIO.OUT, initial=GPIO.LOW)

    # Create LCD and DHT11 objects
    lcd = I2C_LCD_driver.lcd()
    dht11 = adafruit_dht.DHT11(DHT11_PIN, use_pulseio=False)

    # Define functions
    def temp_humi():
        try:
            temperature_c = dht11.temperature
            temperature_f = temperature_c * (9 / 5) + 32
            humidity = dht11.humidity

            # Print values on LCD display
            lcd.lcd_display_string("T:", 1, 0)
            lcd.lcd_display_string(f"{temperature_c:.1f}C", 1, 2)

            lcd.lcd_display_string("H:", 2, 0)
            lcd.lcd_display_string(f"{humidity:.1f}%", 2, 2)

            # Print values to serial port
            logging.info(f"Temp: {temperature_f:.1f} F / {temperature_c:.1f} C    Humidity: {humidity:.1f}%")

            # Control relay based on temperature
            if temperature_c > 32:
                GPIO.output(RELAY_PIN, GPIO.HIGH)
                lcd.lcd_display_string("Relay: ON", 2, 10)
            else:
                GPIO.output(RELAY_PIN, GPIO.LOW)
                lcd.lcd_display_string("Relay: OFF", 2, 10)
        except RuntimeError as error:
            logging.error(error.args[0])
            sleep(1)
        except Exception as error:
            dht11.exit()
            raise error

        sleep(1)

    def get_light():
        value = GPIO.input(LDR_PIN)
        if value == 0:
            lcd.lcd_display_string("L: High", 1, 8)
        else:
            lcd.lcd_display_string("L: LOW ", 1, 8)

    def get_rain():
        bus.write_byte(0x4b, 0x84)  # A0
        value = bus.read_byte(0x4b)
        value = (value / 255) * 100
        value = (value - 100) * -1

        value = int(value)
        lcd.lcd_display_string(f"R: {value}%", 2, 8)

    # Main loop
    while True:
        temp_humi()
        get_light()
        get_rain()

if __name__ == "__main__":
    main()