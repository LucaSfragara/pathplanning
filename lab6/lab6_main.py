import numpy as np
import matplotlib.pyplot as plt
import math
import scipy.stats as stats
import seaborn as sns


class Localization: 
    
    def __init__(self, regions_per_sector:int, block_threshold: float):
        
        self.regions_per_sector = regions_per_sector
        self.N_sectors = 16
        self.total_regions = regions_per_sector * self.N_sectors
        self.block_threshold = block_threshold
        self.angles_per_region = 360 / self.total_regions

        self.probabilities = np.full(self.total_regions, 1.0 / self.total_regions)  # Initialize the probability map (i.e. prior belief) with equal probabilities
        self.previous_region = 0
        
    def motion_model(self, current_angle, sigma_region):
        
        #We defining a PMF for P(new_sector | current_sector, encoder_reading)
        
        """
        Returns P(x' | x, u) for a simple noisy motion on a circle.
        80% chance to move exactly u steps, 10% each for +/-1 step.
        """
        
        #shift probability map by the motion model
        
        current_region = self._get_region_from_angle(current_angle)
        region_shift = current_region - self.previous_region
        
        self.previous_region = current_region

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
    
    
    def bayesian_filter_update(self, bel_prev, distance_reading):
        
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
    
    #blocks_map = {1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0}
    
    loc = Localization(4, 0.5) 
    
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
    
    
