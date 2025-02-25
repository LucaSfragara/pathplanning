import asyncio
import websockets
import json
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

PI_IP = "172.26.229.28"
PORT = 8765

# Initialize data storage
class DataStorage:
    def __init__(self, max_points=1000):
        self.max_points = max_points
        self.times = []
        self.dist = []
        self.prob = []
        self.block_detected = []
        self.start_time = None

data_storage = DataStorage()

# Set up the plot
plt.ion()  # Enable interactive mode
fig, ax1 = plt.subplots()
ax2 = ax1.twinx()  # Create secondary axis

# Create lines for both plots
line_dist, = ax1.plot([], [], 'b-', label='Distance')
line_prob, = ax2.plot([], [], 'r-', label='Probability')

# Setup axes labels and title
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Distance (cm)', color='b')
ax2.set_ylabel('Probability', color='r')
plt.title('Live Sensor Data and Probability')

# Set axis colors
ax1.tick_params(axis='y', labelcolor='b')
ax2.tick_params(axis='y', labelcolor='r')

# Add legend
lines = [line_dist, line_prob]
labels = [l.get_label() for l in lines]
ax1.legend(lines, labels, loc='upper right')

def update_plot():
    if data_storage.times:
        # Update distance plot
        line_dist.set_data(data_storage.times, data_storage.dist)
        # Update probability plot
        line_prob.set_data(data_storage.times, data_storage.prob)
        
        # Adjust limits for both axes
        ax1.relim()
        ax1.autoscale_view()
        ax2.relim()
        ax2.autoscale_view()
        
        # Set probability axis limits
        #ax2.set_ylim(0, 1)  # Probability is always between 0 and 1
        
        fig.canvas.draw()
        fig.canvas.flush_events()

async def receive_data():
    uri = f"ws://{PI_IP}:{PORT}"
    async with websockets.connect(uri) as websocket:
        try:
            data_storage.start_time = asyncio.get_event_loop().time()
            while True:
                data = await websocket.recv()
                sensor_data = json.loads(data)
                current_time = asyncio.get_event_loop().time() - data_storage.start_time
                
                # Parse incoming data
                dist_value = float(sensor_data["dist"])
                prob_value = float(sensor_data["prob"])
                block_detected = sensor_data["block_detected"]
                
                # Append new data
                data_storage.times.append(current_time)
                data_storage.dist.append(dist_value)
                data_storage.prob.append(prob_value)
                data_storage.block_detected.append(block_detected)
                
                # Keep only the last max_points for all arrays
                if len(data_storage.times) > data_storage.max_points:
                    data_storage.times.pop(0)
                    data_storage.dist.pop(0)
                    data_storage.prob.pop(0)
                    data_storage.block_detected.pop(0)
                
                update_plot()
                print(f"Distance: {dist_value:.4f}, Probability: {prob_value:.4f}, block detected: {block_detected}")
                
        except websockets.exceptions.ConnectionClosedOK:
            print("Server closed the connection.")
        except websockets.exceptions.ConnectionClosedError:
            print("Unexpected disconnection from server.")

if __name__ == "__main__":
    asyncio.run(receive_data())


