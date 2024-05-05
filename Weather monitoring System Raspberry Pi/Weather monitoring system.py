# Include the library files
import I2C_LCD_driver
from time import sleep
import board
import adafruit_dht
from smbus import SMBus
import RPi.GPIO as GPIO


LDR = 27

bus = SMBus(1)

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Set the sensor pin as Input pin
GPIO.setup(LDR,GPIO.IN)

# Create a object for the LCD
lcd = I2C_LCD_driver.lcd()
# Create a object for the DHT11 sensor
DHT11 = adafruit_dht.DHT11(board.D17, use_pulseio=False)

# Starting text
lcd.lcd_display_string("System Loading",1,1)
for a in range (0,16):
    lcd.lcd_display_string(".",2,a)
    sleep(0.1)
lcd.lcd_clear()
lcd.lcd_display_string("Wait..",1,0)

def TempHumi():
    try:
        # Get the Temperature and Humidity values
        temperature_c = DHT11.temperature
        temperature_f = temperature_c * (9 / 5) + 32
        humidity = DHT11.humidity
        
        
        # Print the values on the LCD display
        lcd.lcd_display_string("T:",1,0)
        lcd.lcd_display_string(str(temperature_c) + ".0C",1,2)
        
        lcd.lcd_display_string("H:",2,0)
        lcd.lcd_display_string(str(humidity) + "%",2,2)
        
        # Print the values to the serial port
        print(
            "Temp: {:.1f} F / {:.1f} C    Humidity: {}% ".format(
                temperature_f, temperature_c, humidity
            )
        )
 
    except RuntimeError as error:
        # Errors happen fairly often, DHT's are hard to read, just keep going
        print(error.args[0])
        sleep(1)
    except Exception as error:
        DHT11.exit()
        raise error
 
    sleep(1)
    
def light():
    value = GPIO.input(LDR)
    if value == 0:
        lcd.lcd_display_string("L:" + "High",1,8)
    else:
        lcd.lcd_display_string("L:" + "LOW ",1,8)
        
#Get the analog input values
def rain():
    bus.write_byte(0x4b,0x84)# A0
    value = bus.read_byte(0x4b)
    value = (value/255) * 100
    value = (value - 100) * -1
    
    value = int(value)
    lcd.lcd_display_string("R:" + str(value) + "% " ,2,8)
    
         
    
while True:
    TempHumi()
    light()
    rain()
    