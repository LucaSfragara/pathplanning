import numpy as np
import matplotlib.pyplot as plt
import math
import scipy.stats as stats
import seaborn as sns


class Localization: 
    
    def __init__(self, regions_per_sector:int, block_threshold_lower: float, block_threshold_upper: float, blocks_map: list):
        
        self.regions_per_sector = regions_per_sector
        self.N_sectors = 16
        self.total_regions = regions_per_sector * self.N_sectors
        self.angles_per_region = 360 / self.total_regions

        self.probabilities = np.full(self.total_regions, 1.0 / self.total_regions)  # Initialize the probability map (i.e. prior belief) with equal probabilities
        self.current_region = 0
        
        #closest block is around 8, farthest around 28, maybe adjust this
        self.block_threshold_lower = 5
        self.block_threshold_upper = 35
        
        #repeat each element of the list 4 times mantaining the order
        self.blocks_map = []
        for el in self.map:
            self.blocks_map.extend([el] * 4)
        
        
    def motion_model(self, current_angle, sigma_region):
        
        #We defining a PMF for P(new_sector | current_sector, encoder_reading)
        
        """
        Returns P(x' | x, u) for a simple noisy motion on a circle.
        80% chance to move exactly u steps, 10% each for +/-1 step.
        """
        
        #shift probability map by the motion model
        
        current_region = self._get_region_from_angle(current_angle)
        region_shift = current_region - self.current_region
        
        self.current_region = current_region

        region_centers = np.arange(0, self.total_regions) 

        
        if region_shift == 0:
            return self.probabilities, region_centers
        
        shifted_map = np.roll(self.probabilities, region_shift)
    
        region_centers = np.arange(0, self.total_regions) 
    
        gaussian_weights = stats.norm.pdf(region_centers, loc = current_region, scale = sigma_region)
        gaussian_weights = gaussian_weights / np.sum(gaussian_weights)
        
        noisy_prob_map = shifted_map * gaussian_weights
        noisy_prob_map /= np.sum(noisy_prob_map)  # Normalize again

        self.probabilities = noisy_prob_map
        
        return  noisy_prob_map, gaussian_weights
        
    
    def _get_region_from_angle(self, angle):
        
        if angle < 0 or angle > 360: 
            raise ValueError("Angle must be between 0 and 360")
        
        # Returns the corresponding sector of the angle
        return int(angle / self.angles_per_region)    

    def _get_sector_from_region(self, region):
        
        if region < 0 or region > self.N_sectors * self.regions_per_sector: 
            raise ValueError("Region must be between 0 and 64")
        
        # Returns the corresponding sector of the angle
        return int(region / self.regions_per_sector)
    

    def sensor_model(self, distance_reading, d_expected, sigma=0.5):     
        """
        Returns P(z | x) for a sensor model with Gaussian noise.
        Params: 
        - distance_readings: the distance reading from the sensor
        - sigma: the standard deviation of the sensor noise
        - d_expected: the expected distance for block detection
        """
        block_detected = False   
        
        if distance_reading < self.block_threshold_upper and distance_reading > self.block_threshold_lower:
            block_detected = True
            
        # Calculate probability using Gaussian distribution
        if block_detected:
            # If block detected, calculate probability based on how close to expected
            prob_block = stats.norm.pdf(distance_reading, d_expected, sigma)
            return prob_block if self.blocks_map[self.current_region] == 1 else (1 - prob_block)
        else:
            # If no block detected, inverse probability
            prob_no_block = 1 - stats.norm.pdf(distance_reading, d_expected, sigma)
            return prob_no_block if self.blocks_map[self.current_region] == 0 else (1 - prob_no_block)
    
    def bayesian_filter_update(self, distance_reading, sigma=0.5):
        """
        Updated Bayesian filter incorporating sensor noise
        """
        # Prediction step (remains unchanged)
        bel_bar = self.probabilities  # Using current probabilities as prior
        
        # Update step with sensor model
        bel = np.zeros(self.total_regions)
        for x_prime in range(self.total_regions):
            # Calculate sensor probability for each region
            sensor_prob = self.sensor_model(distance_reading, sigma)
            bel[x_prime] = sensor_prob * bel_bar[x_prime]
            
        # Normalize
        total_sum = np.sum(bel)
        if total_sum > 0:
            bel = bel / total_sum
        
        self.probabilities = bel
        return bel

if __name__ == "__main__":
    
    blocks_map = {1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0}
    
    
    #loc = Localization(4, 0.5, block_threshold_upper=) 
    
    #region = loc._get_region_from_angle(360)
    #sector = loc._get_sector_from_region(region)
    
    #print(f"region: {region}, sector: {sector}")
    
    for angle in range(0, 180):
        
        noisy_prob_map, gaussian_weights = loc.motion_model(current_angle=angle, sigma_region=5) #SD. in regions


    #print(gaussian_weights)
    sns.barplot(x = np.arange(0, loc.total_regions), y = noisy_prob_map)
    
    #reduce number of ticks
    plt.xticks(np.linspace(0, loc.total_regions,loc.total_regions // 4 ))
    
    plt.show()


