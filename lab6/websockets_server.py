import asyncio
import websockets
import json
import random  # Simulated sensor data
from tof import read_data
<<<<<<< Updated upstream
=======
from localization import Localization
>>>>>>> Stashed changes

# WebSocket server settings
HOST = "0.0.0.0"  # Listen on all interfaces
PORT = 8765       # Any available port

# WebSocket handler (updated to match newer websockets API)
async def sensor_data(websocket):  # Removed 'path' parameter
    try:
        while True:
            # Simulated sensor data
<<<<<<< Updated upstream
            message = json.dumps({
                "value": read_data()
            })
=======
            
            blocks_map = [1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    
    
            loc = Localization(4, block_threshold_upper=35, block_threshold_lower=5, blocks_map = blocks_map) 
            dist = read_data()
            if dist > 1:
                prob, block_detected = loc.sensor_model(dist, 17, sigma = 5)
                message = json.dumps({
                    "dist": dist, 
                    "prob": prob, 
                    "block_detected": block_detected
                })
>>>>>>> Stashed changes
            await websocket.send(message)
            await asyncio.sleep(0.1)

    except websockets.exceptions.ConnectionClosedOK:
        print("Client disconnected normally.")
    except websockets.exceptions.ConnectionClosedError:
        print("Client disconnected unexpectedly.")

# WebSocket server initialization
async def main():
    async with websockets.serve(sensor_data, HOST, PORT):
        print(f"WebSocket server started on ws://{HOST}:{PORT}")
        await asyncio.Future()  # Keep the server running

# Run the WebSocket server
if __name__ == "__main__":
    asyncio.run(main())
