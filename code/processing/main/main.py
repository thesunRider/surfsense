import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

#check https://github.com/balamuruganky/EKF_IMU_GPS/tree/master

data = pd.read_csv('test_data.csv')
print(data.head()) # to display the first 5 lines of loaded data
data["timestamp"] = data["timestamp"]- data["timestamp"][0]
data["IMU-ticks"] = data["IMU-ticks"]- data["IMU-ticks"][0]
data["GPS-ticks"] = data["GPS-ticks"]- data["GPS-ticks"][0]
data["pressure-ticks"] = data["pressure-ticks"]- data["pressure-ticks"][0]

data['lat'] = data['lat'] / 1e7
data['long'] = data['long'] / 1e7

print(data)

data.plot(x='GPS-ticks', y='lat', title='lat over Time', figsize=(10, 4))
plt.xlabel("Timestamp (ms)")
plt.ylabel("Latitude")
plt.grid(True)
plt.show()



data.plot(x='GPS-ticks', y='long', title='lat over Time', figsize=(10, 4))
plt.xlabel("Timestamp (ms)")
plt.ylabel("Latitude")
plt.grid(True)
plt.show()

colors = data['GPS-ticks']
fig, ax = plt.subplots(figsize=(10, 6))
sc = ax.scatter(data['lat'], data['long'], c=colors, cmap='viridis', s=10)

# Add colorbar for reference
cbar = plt.colorbar(sc, ax=ax)
cbar.set_label('GPS Ticks')
plt.show()
