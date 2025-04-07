import serial
from influxdb import InfluxDBClient

# Paramètres de connexion à InfluxDB
influx_client = InfluxDBClient(host='localhost', port=8086)
influx_client.switch_database('robot')  # Assure-toi que la base de données 'robot' existe

# Paramètres du port série
ser = serial.Serial('COM4', 115200)  # Remplace 'COM3' par le port série utilisé par ton Arduino

while True:
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').strip()
        
        try:
            roll, pitch = map(float, line.split(','))
            
            # Créer un dictionnaire de données à insérer dans InfluxDB
            json_body = [
                {
                    "measurement": "sensor_data",
                    "tags": {
                        "device": "mpu6050"
                    },
                    "fields": {
                        "roll": roll,
                        "pitch": pitch
                    }
                }
            ]
            
            # Insérer les données dans InfluxDB
            influx_client.write_points(json_body)
            print(f"Data sent to InfluxDB: Roll={roll}, Pitch={pitch}")
        except ValueError:
            print("Invalid data received.")
