import numpy as np
#import matplotlib.pyplot as plt
import math
import scipy.stats as stats
#import seaborn as sns
from tof import read_data
import time

class Localization: 
    
    def __init__(self, regions_per_sector:int, block_threshold_upper: float, block_threshold_lower: float, blocks_map: list):
        
        self.regions_per_sector = regions_per_sector
        self.N_sectors = 16
        self.total_regions = regions_per_sector * self.N_sectors
        self.angles_per_region = 360 / self.total_regions

        self.probabilities = np.full(self.total_regions, 1.0 / self.total_regions)  # Initialize the probability map (i.e. prior belief) with equal probabilities
        self.current_region = 0
        
        #closest block is around 8, farthest around 28, maybe adjust this
        self.block_threshold_lower = block_threshold_lower
        self.block_threshold_upper = block_threshold_upper
        
        #repeat each element of the list 4 times mantaining the order
        self.blocks_map = []
        for el in blocks_map:
            self.blocks_map.extend([el] * self.regions_per_sector)
        
        
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
    

    def sensor_model(self, distance_reading, d_expected, sigma):     
        """
        Returns P(z | x) for a sensor model with Gaussian noise.
        Params: 
        - distance_readings: the distance reading from the sensor
        - sigma: the standard deviation of the sensor noise
        - d_expected: the expected distance for block detection
        """
        block_detected = False   
        print(distance_reading)
        if distance_reading < self.block_threshold_upper and distance_reading > self.block_threshold_lower:
            block_detected = True
            
        # Calculate probability using Gaussian distribution
        if block_detected:
            # If block detected, calculate probability based on how close to expected
            print("block detected")
            prob_block = stats.norm.cdf(distance_reading, d_expected, sigma)
            return prob_block if self.blocks_map[self.current_region] == 1 else (1 - prob_block), block_detected
        else:
            # If no block detected, inverse probability
            print("no block detected")
            prob_no_block = 1 - stats.norm.cdf(distance_reading, d_expected, sigma)
            print(prob_no_block)
            return prob_no_block if self.blocks_map[self.current_region] == 0 else (1 - prob_no_block), block_detected
    
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
    
    def integrated_update(self, distance_reading, current_angle, sigma_region=5, sigma_sensor=2, d_expected=23):
        """
        Integrated update that first performs the motion update and then incorporates
        the sensor measurement.
        
        Parameters:
          - current_angle: the new angle (from e.g. an encoder)
          - distance_reading: the measured distance from the sensor
          - sigma_region: standard deviation for the motion (region) noise
          - sigma_sensor: standard deviation for the sensor noise
          - d_expected: the expected distance when a block is present
        """
        # predicted probabilities based on motion update
        predicted_bel = self.motion_model(current_angle, sigma_region)
        
        # initialize array for sensor update
        updated_bel = np.zeros_like(predicted_bel)
        
        # check if block detected
        block_detected = self.block_threshold_lower < distance_reading < self.block_threshold_upper
        
        #if a block is detected, then regions expected to have a block (blocks_map == 1)
        #should have a high likelihood, else, the likelihood is low.
        for region in range(self.total_regions):
            expected_block = self.blocks_map[region]
            if block_detected:
                # When a block is detected, use the CDF to see how close the reading is to d_expected.
                prob = stats.norm.cdf(distance_reading, loc=d_expected, scale=sigma_sensor)
                # If a block is expected, likelihood is higher when prob is high.
                likelihood = prob if expected_block == 1 else (1 - prob)
            else:
                # When no block is detected, use the inverse probability.
                prob = 1 - stats.norm.cdf(distance_reading, loc=d_expected, scale=sigma_sensor)
                likelihood = prob if expected_block == 0 else (1 - prob)
                
            updated_bel[region] = predicted_bel[region] * likelihood
        
        #normalize
        if updated_bel.sum() > 0:
            updated_bel /= updated_bel.sum()
        else:
            # In case of a numerical error, revert to the predicted belief.
            updated_bel = predicted_bel
        
        self.probabilities = updated_bel
        return updated_bel

if __name__ == "__main__":
    
    blocks_map = [1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    
    
    loc = Localization(4, block_threshold_upper=35, block_threshold_lower=5, blocks_map = blocks_map) 
    
    #region = loc._get_region_from_angle(360)
    #sector = loc._get_sector_from_region(region)
    
    #print(f"region: {region}, sector: {sector}")
    
    #for angle in range(0, 180):
        
    #    noisy_prob_map, gaussian_weights = loc.motion_model(current_angle=angle, sigma_region=5) #SD. in regions
    start_time = time.time()
    while True:
        loop_time = time.time()-start_time()

        dist = read_data()
        if dist > 1: #not a bad value

            #integrated update combines motion update with sensor update, CHANGE CURRENT ANGLE
            belief = loc.integrated_update(dist, current_angle=10, sigma_region=5, sigma_sensor=2, d_expected=23)
            
            #gets index of most probable region
            most_probable_region = np.argmax(belief)
            #get target angle of most likely region
            target_angle = most_probable_region * loc.angles_per_region

            #if we are adequately sure about best region and we've done at least 1 loop and were at the goal, STOP
            if belief[most_probable_region] > 0.6 and loop_time > 30 and  most_probable_region == loc.current_region:
                print(f"Probability {belief[most_probable_region]}")
                break
                
            #print(loc.integrated_update(dist, current_angle=10, sigma_region=5, sigma_sensor=2, d_expected=23))
            time.sleep(0.1)




        #print(loc.sensor_model(dist, 23))
    #print(gaussian_weights)
    #sns.barplot(x = np.arange(0, loc.total_regions), y = noisy_prob_map)
    
    #reduce number of ticks
    #plt.xticks(np.linspace(0, loc.total_regions,loc.total_regions // 4 ))
    
    #plt.show()


