import serial
import csv

def parse_sns_data(data):
    # Remove markers and split the data
    data = data.strip('>>sns dat:').strip('<<')
    values = data.split(':')
    
    if len(values) != 16:
        print("Warning: Unexpected data format!", data)
        return None
    
    labels = [
        "timestamp", "accel_X", "accel_Y", "accel_Z",
        "gyro_X","gyro_Y", "gyro_Z", "mag_X", "mag_Y", "mag_Z",
        "temperature", "latitude", "longitude", "altitude", "siv", "force"
    ]
    
    parsed_data = dict(zip(labels, values))
    return parsed_data

def read_serial(port="COM3", baudrate=115200, output_file="sensor_data.csv"):
    ser = serial.Serial(port, baudrate, timeout=1)
    print(f"Listening on {port}...")
    
    with open(output_file, "w", newline='') as csvfile:
        fieldnames = [
            "timestamp", "accel_X", "accel_Y", "accel_Z",
            "gyro_X","gyro_Y", "gyro_Z", "mag_X", "mag_Y", "mag_Z",
            "temperature", "latitude", "longitude", "altitude", "siv", "force"
        ]
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        
        try:
            while True:
                line = ser.readline().decode('utf-8').strip()
                if line.startswith(">>sns dat:"):
                    parsed_data = parse_sns_data(line)
                    if parsed_data:
                        writer.writerow(parsed_data)
                        print(parsed_data)
        except KeyboardInterrupt:
            print("Stopping serial read.")
        finally:
            ser.close()

if __name__ == "__main__":
    read_serial(port="COM9", baudrate=115200)  # Change for your setup
