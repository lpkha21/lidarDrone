import serial
import threading
import queue
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from collections import deque

# Initialize serial communication
ser = serial.Serial('COM9', 250000, timeout=1)

# Queue to store incoming data
data_queue = queue.Queue()

# Flag to stop the threads when needed
running = threading.Event()
running.set()

# Filter settings
DISTANCE_MIN = 0.1  # Minimum distance (in meters)
DISTANCE_MAX = 1000.0  # Maximum distance (in meters)

# Use deques with a maximum length to store processed data
MAX_POINTS = 100000  # Adjust this value based on your memory constraints
x_coords = deque(maxlen=MAX_POINTS)
y_coords = deque(maxlen=MAX_POINTS)
z_coords = deque(maxlen=MAX_POINTS)

# Drone settings
drone_altitude = 1000  # Drone altitude in meters
drone_position_y = 0  # Y position of the drone

# Lock for thread-safe operations on shared data
data_lock = threading.Lock()

drone_position_y_lock = threading.Lock()

# Thread 1: Serial reader
def serial_reader():
    while running.is_set():
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            for byte in data:
                data_queue.put(byte)

# Thread 2: Data processor
def data_processor():
    global drone_position_y
    global x_coords
    global y_coords
    global z_coords
    counter = 0
    while running.is_set():
        try:
            if data_queue.qsize() >= 9:
                ph1 = data_queue.get(timeout=1)
                ph2 = data_queue.get(timeout=1)
                if ph1 == 0xAA and ph2 == 0x55:
                    ct = data_queue.get(timeout=1)
                    ls = data_queue.get(timeout=1)
                    fsa2 = data_queue.get(timeout=1)
                    fsa1 = data_queue.get(timeout=1)
                    lsa2 = data_queue.get(timeout=1)
                    lsa1 = data_queue.get(timeout=1)
                    cs2 = data_queue.get(timeout=1)
                    cs1 = data_queue.get(timeout=1)

                    fsa = (fsa1 << 8) | fsa2
                    lsa = (lsa1 << 8) | lsa2
                    startAngle = (fsa >> 1) / 64.0
                    endAngle = (lsa >> 1) / 64.0

                    new_x = []
                    new_y = []
                    new_z = []
                    

                    for i in range(ls):
                        s1 = data_queue.get(timeout=1)
                        s2 = data_queue.get(timeout=1)
                        s3 = data_queue.get(timeout=1)
                        distance = ((s3 << 16) | (s2 << 8) | s1) / 1000.0  # Convert to meters
                        angle = (endAngle - startAngle) / ls * i + startAngle

                        if DISTANCE_MIN <= distance <= DISTANCE_MAX and (angle <= 60 or angle >= 300):
                            if angle >= 300:
                                angle = 360 - angle
                                new_x.append(-(distance * np.sin(np.radians(angle))))
                            else:
                                new_x.append(distance * np.sin(np.radians(angle)))
                            new_y.append(drone_position_y)
                            new_z.append(drone_altitude - distance * np.cos(np.radians(angle)))

                    # Update the global lists thread-safely
                    with data_lock:
                        x_coords.extend(new_x)
                        y_coords.extend(new_y)
                        z_coords.extend(new_z)
                        if counter == 10:
                            x_coords = deque(list(x_coords)[500:])
                            y_coords = deque(list(y_coords)[500:])
                            tmp = y_coords[i]
                            for i in range(len(y_coords)):
                                y_coords[i] = y_coords[i]-tmp
                            z_coords = deque(list(z_coords)[500:])
                            counter = 0

                        
                    
                    if ls == 1:
                        counter += 1
                        with drone_position_y_lock:
                            drone_position_y -= 0.1

        except queue.Empty:
            pass
        except Exception as e:
            print(f"Error in data processing: {e}")

# Create and start threads
reader_thread = threading.Thread(target=serial_reader)
processor_thread = threading.Thread(target=data_processor)

reader_thread.start()
processor_thread.start()

# Set up the 3D plot
plt.ion()  # Turn on interactive mode
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Initialize the scatter plot
scatter = ax.scatter([], [], [], s = 10)

# Set labels for axes
ax.set_xlabel('X (meters)')
ax.set_ylabel('Y (meters)')
ax.set_zlabel('Z (meters)')

# Add colorbar

# Main program loop with visualization
try:
    while running.is_set():
        with data_lock:
            if x_coords:
                # Convert deques to numpy arrays for plotting
                x = np.array(x_coords)
                y = np.array(y_coords)
                z = np.array(z_coords)

                # Update the scatter plot data
                scatter._offsets3d = (x, y, z)
                scatter.set_array(z)

                # Update plot limits if necessary
                ax.set_xlim(min(x.min(), ax.get_xlim()[0]), max(x.max(), ax.get_xlim()[1]))
                ax.set_ylim(min(y.min(), ax.get_ylim()[0]), max(y.max(), ax.get_ylim()[1]))
                ax.set_zlim(min(z.min(), ax.get_zlim()[0]), max(z.max(), ax.get_zlim()[1]))

                # Update the colorbar limits
                scatter.set_clim(z.min(), z.max())
                
        # Redraw the plot
        fig.canvas.draw_idle()
        plt.pause(0.1)

except KeyboardInterrupt:
    # Stop all threads when Ctrl+C is pressed
    running.clear()
    reader_thread.join()
    processor_thread.join()

    # Clean up serial connection
    ser.close()

    # Close all pyplot windows
    plt.close('all')

print("Program terminated.")