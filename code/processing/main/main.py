import pandas as pd
import matplotlib.pyplot as plt

data = pd.read_csv('test_data.csv')
print(data.head()) # to display the first 5 lines of loaded data
data["timestamp"] = data["timestamp"]- data["timestamp"][0]
data["IMU-ticks"] = data["IMU-ticks"]- data["IMU-ticks"][0]
data["GPS-ticks"] = data["GPS-ticks"]- data["GPS-ticks"][0]

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