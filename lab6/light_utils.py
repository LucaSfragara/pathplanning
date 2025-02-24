import time
import board
import adafruit_bh1750

i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
sensor = adafruit_bh1750.BH1750(i2c)


def get_light():
    
    """
    Reads and returns the ambient light intensity in lux.

    This function retrieves the current light level using the BH1750 light sensor.
    The BH1750 sensor measures the ambient light intensity and returns the value in lux.

    Returns:
        float: The measured light intensity in lux.
    """
    return sensor.lux

def is_line(threshold: int): 
    
    """
    Determines whether the detected light intensity meets or exceeds a given threshold.

    This function compares the current ambient light intensity, as measured by the
    BH1750 sensor, against a specified threshold. If the measured light intensity
    is greater than or equal to the threshold, it returns True; otherwise, it returns False.

    Args:
        threshold (int): The light intensity threshold in lux.

    Returns:
        bool: True if the measured light intensity is greater than or equal to the threshold, 
              False otherwise.
    """
    
    return _get_light() >= threshold
    


if __name__ == "__main__":
    while True: 
        print("light from light utils: ", sensor.lux)
        time.sleep(0.1)