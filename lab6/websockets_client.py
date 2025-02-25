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
        self.sensor_values = []
        self.start_time = None

data_storage = DataStorage()

# Set up the plot
plt.ion()  # Enable interactive mode
fig, ax = plt.subplots()
line, = ax.plot([], [], 'b-')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Sensor Value')
ax.set_title('Live Sensor Data')
ax.grid(True)

def update_plot():
    if data_storage.times and data_storage.sensor_values:
        line.set_data(data_storage.times, data_storage.sensor_values)
        ax.relim()
        ax.autoscale_view()
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
                
                # Assuming sensor_data contains a 'value' field - adjust as needed
                sensor_value = float(sensor_data.get('value', 0))
                if sensor_value == -1:
                    continue
                data_storage.times.append(current_time)
                data_storage.sensor_values.append(sensor_value)
                
                # Keep only the last max_points
                if len(data_storage.times) > data_storage.max_points:
                    data_storage.times.pop(0)
                    data_storage.sensor_values.pop(0)
                
                update_plot()
                print(f"Received Sensor Data: {sensor_data}")
                
        except websockets.exceptions.ConnectionClosedOK:
            print("Server closed the connection.")
        except websockets.exceptions.ConnectionClosedError:
            print("Unexpected disconnection from server.")

if __name__ == "__main__":
    asyncio.run(receive_data())


