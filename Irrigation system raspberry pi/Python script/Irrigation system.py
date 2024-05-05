# Include the library files
import I2C_LCD_driver
from time import sleep
from smbus import SMBus
import RPi.GPIO as GPIO

bus = SMBus(1)

button = 17
relay = 27
motorstatus = True

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Set the button pin as Input pin
GPIO.setup(button,GPIO.IN,pull_up_down=GPIO.PUD_UP)

#Set the relay pin as output pin
GPIO.setup(relay,GPIO.OUT)

GPIO.output(relay,GPIO.HIGH) # Relay turns OFF

# Create a object for the LCD
lcd = I2C_LCD_driver.lcd()

# Starting text
lcd.lcd_display_string("System Loading",1,1)
for a in range (0,16):
    lcd.lcd_display_string(".",2,a)
    sleep(0.1)
lcd.lcd_clear()

lcd.lcd_display_string("Motor   :OFF" ,2,0)
          
#Get the analog input values
def moistureValue():
    bus.write_byte(0x4b,0x84)# A0
    value = bus.read_byte(0x4b)
    value = (value/255) * 100
    value = (value - 100) * -1
    
    value = int(value)
    lcd.lcd_display_string("Moisture:" + str(value) + "%  " ,1,0)
      
    
while True:
    moistureValue()
    if GPIO.input(button) == 0: # Get the button value
        if motorstatus:
            GPIO.output(relay,GPIO.LOW) # Relay turns ON
            lcd.lcd_display_string("Motor   :ON " ,2,0)
            sleep(0.5)
            motorstatus = False
            
        elif motorstatus == False:
            GPIO.output(relay,GPIO.HIGH) # Relay turns OFF
            lcd.lcd_display_string("Motor   :OFF" ,2,0)
            sleep(0.5)
            motorstatus = True
       
    




    
