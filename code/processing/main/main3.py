import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from ahrs import Quaternion
import ahrs
from ahrs.common.orientation import q2R
import os

import imufusion


def list_csv_files(directory):
    csv_files = []
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.lower().endswith('.csv'):
                csv_files.append(os.path.join(root, file))
    return csv_files

# Example usage
directory_path = '..\\..\\..\\test_data\\walks\\outside_elab\\new'
csv_files = list_csv_files(directory_path)

#check https://github.com/balamuruganky/EKF_IMU_GPS/tree/master

data = pd.read_csv(csv_files[-1])
print(data.head()) # to display the first 5 lines of loaded data
data["timestamp"] = data["timestamp"]- data["timestamp"][0]
data["IMU-ticks"] = data["IMU-ticks"]- data["IMU-ticks"][0]
data["GPS-ticks"] = data["GPS-ticks"]- data["GPS-ticks"][0]
data["pressure-ticks"] = data["pressure-ticks"]- data["pressure-ticks"][0]

data['lat'] = data['lat'] / 1e7
data['long'] = data['long'] / 1e7

print(data)



colors = data['GPS-ticks']
fig, ax = plt.subplots(nrows=3, ncols=3,figsize=(10, 6))
sc = ax[0,0].scatter(data['lat'], data['long'], c=colors, cmap='viridis', s=10)
ax[0,0].set_title("lat vs long")

ax[0,1].scatter(data["IMU-ticks"], data['aX'],c=data["IMU-ticks"],cmap='viridis', s=10,label='SCATTER')
ax[0,1].set_title("aX")

ax[0,2].scatter(data["IMU-ticks"], data['aY'],c=data["IMU-ticks"],cmap='viridis', s=10)
ax[0,2].set_title("aY")

ax[1,0].scatter(data["IMU-ticks"], data['aZ'],c=data["IMU-ticks"],cmap='viridis', s=10)
ax[1,0].set_title("aZ")

ax[1,1].scatter(data["IMU-ticks"], data['gX'],c=data["IMU-ticks"],cmap='viridis', s=10)
ax[1,1].set_title("gX")

ax[1,2].scatter(data["IMU-ticks"], data['gY'],c=data["IMU-ticks"],cmap='viridis', s=10)
ax[1,2].set_title("gY")

ax[2,0].scatter(data["IMU-ticks"], data['gZ'],c=data["IMU-ticks"],cmap='viridis', s=10)
ax[2,0].set_title("gZ")

ax[2,1].scatter(data["IMU-ticks"],data['mX'] ,c=data["IMU-ticks"],cmap='viridis', s=10)
ax[2,1].set_title("mX")

ax[2,2].scatter(data["IMU-ticks"],data['mY'] ,c=data["IMU-ticks"],cmap='viridis', s=10)
ax[2,2].set_title("mY")

# Add colorbar for reference
cbar = plt.colorbar(sc, ax=ax)
cbar.set_label('GPS Ticks')

#plt.show()


acc = data[['aX', 'aY', 'aZ']].to_numpy() #m/s2
gyr = data[['gX', 'gY', 'gZ']].to_numpy() #rad/s
mag = data[['mX', 'mY', 'mZ']].to_numpy() #uT

mag /= 1000 # the library expects mT
ts = data['IMU-ticks'].to_numpy()
dt = np.diff(ts) / 1000.0  # Convert ms to seconds
dt = np.append(dt, dt[-1])  # Same length as data

# Process sensor data
ahrs = imufusion.Ahrs()
euler = np.empty((len(dt), 3))

for index in range(len(dt)):
    ahrs.update_no_magnetometer(gyr[index], acc[index], np.mean(dt))  # 100 Hz sample rate
    euler[index] = ahrs.quaternion.to_euler()

print(euler)

breakpoint()

# Remove gravity to get linear acceleration in world frame
positions = [np.zeros(3)]
velocities = [np.zeros(3)]

for i in range(1, len(acc)):
    lin_acc =  euler[i] acc[i]         # Rotate to world frame
    lin_acc -= [0, 0, 9.81]      # Remove gravity

    v = velocities[-1] + lin_acc * dt[i]
    p = positions[-1] + v * dt[i]

    velocities.append(v)
    positions.append(p)

positions = np.array(positions)
# Plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(positions[:,0], positions[:,1], positions[:,2])
ax.set_title("Estimated Position from IMU")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
plt.show()

