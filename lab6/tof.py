
# Simple demo of the VL53L4CD distance sensor.
# Will print the sensed range/distance every second.
import board
import adafruit_vl53l4cx

i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller

vl53 = adafruit_vl53l4cx.VL53L4CX(i2c)

print("VL53L4CX Simple Test.")

vl53.start_ranging()

def read_data(): 
    
    if vl53.data_ready:
        vl53.clear_interrupt()
        return vl53.distance
    else: 
        return -1
    

if __name__ == "__main__":
    while True:
        r = read_data()
        if r > 0:
            print(r)
    

