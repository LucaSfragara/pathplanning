import numpy as np
import matplotlib.pyplot as plt
import math


N = 32 #We have 16 sectors but we discretize the cirle in smaller chunks
BLOCK_THRESHOLD = 0.5 #If the distance reading is lower than this threshold, we consider it as a block


class Localization: 
    
    def __init__(self, blocks_map,  N =  N, block_threshold = BLOCK_THRESHOLD, current_sector = 0):
        
        self.blocks_map = blocks_map
        self.N = N
        self.block_threshold = block_threshold
        self.bel= np.array((N), fill_value = 1/N) #Initialize the probability map (i.e. prior belief) with equal probabilities
        self.current_sector = current_sector
        
    def motion_model(self, x_prime, x, u):
        
        #We defining a PMF for P(new_sector | current_sector, encoder_reading)
        
        """
        Returns P(x' | x, u) for a simple noisy motion on a circle.
        80% chance to move exactly u steps, 10% each for +/-1 step.
        """
        # compute the main position
        main = (x + u) % N
        left = (x + u - 1) % N
        right = (x + u + 1) % N
        
        if x_prime == main:
            return 0.8
        elif x_prime == left or x_prime == right:
            return 0.1
        else:
            return 0.0

        

    def sensor_model(self, distance_readings, z, sigma = 0.5, d_expected = 1):     
        
        """
        Returns P(z | x) for a  sensor model.
        Params: 
        - distance_readings: the distance reading from the sensor
        - sigma: the standard deviation of the sensor
        - d_expected: the expected distance
        """
           
        coeff = 1.0 / (math.sqrt(2.0 * math.pi) * sigma)
        exponent = -0.5 * ((z - d_expected) ** 2) / (sigma ** 2)
        return coeff * math.exp(exponent)
    
    
    def bayesia_filter_update(self, bel_prev, distance_reading):
        
        #prediction (Model Update). We shift the probability map by the motion model
        
        
        bel_bar = np.zeros((self.N))
        for x_prime in range(self.N):
            total = 0
            for x in range(self.N):
                total += self.motion_model(x_prime, x) * bel_prev[x]
            
                 
        #Update the probability map based on the sensor model
        
        bel = np.zeros((self.N))
        for x_prime in range(self.N):
            bel[x_prime] = self.sensor_model(distance_reading) * bel_bar[x_prime]
            
            
        #normalize the new probability map
        total_sum = np.sum(bel)
        if total_sum != 0:
            bel = bel / total_sum
        
        return bel
    
if __name__ == "__main__":
    
    blocks_map = {1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0}
    loc = Localization(blocks_map) 
