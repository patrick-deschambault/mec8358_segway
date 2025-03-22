import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Configuration du port série (remplace 'COM4' par ton port si besoin)
ser = serial.Serial('COM4', 115200, timeout=1)

# Initialisation du graphique
fig, ax = plt.subplots()
ax.set_ylim(-180, 180)  # Plage des angles
ax.set_xlim(0, 100)  # Nombre de points affichés
roll_data, pitch_data, yaw_data = [], [], []
x_data = list(range(100))

# Fonction de mise à jour du graphique
def update(frame):
    try:
        line = ser.readline().decode().strip()  # Lire une ligne et nettoyer
        values = line.split()  # Séparer les valeurs par espace
        
        if len(values) == 3:  # Vérifier qu'on a bien 3 valeurs
            roll, pitch, yaw = map(float, values)  # Convertir en float
            roll_data.append(roll)
            pitch_data.append(pitch)
            yaw_data.append(yaw)
            
            if len(roll_data) > 100:  # Garder les 100 dernières valeurs
                roll_data.pop(0)
                pitch_data.pop(0)
                yaw_data.pop(0)
            
            ax.clear()
            ax.set_ylim(-180, 180)
            ax.set_xlim(0, 100)
            ax.plot(x_data[-len(roll_data):], roll_data, label="Roll", color='r')
            ax.plot(x_data[-len(pitch_data):], pitch_data, label="Pitch", color='g')
            ax.plot(x_data[-len(yaw_data):], yaw_data, label="Yaw", color='b')
            ax.legend()
    except Exception as e:
        print(f"Erreur : {e}")  # Debug si problème

# Correction du warning
ani = animation.FuncAnimation(fig, update, interval=50, cache_frame_data=False)

plt.show()
