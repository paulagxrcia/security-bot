import csv
import matplotlib.pyplot as plt

# Leer datos del archivo CSV
timestamps = []
positions = []
efforts = []

with open('joint_log.csv', 'r') as file:
    reader = csv.DictReader(file)
    for row in reader:
        timestamps.append(float(row['time']))
        positions.append(float(row['position']))
        efforts.append(float(row['effort']))

# Crear las gráficas
plt.figure(figsize=(10, 6))

# Posición
plt.subplot(2, 1, 1)
plt.plot(timestamps, positions, label='Posición [m]')
plt.ylabel('Posición del mástil')
plt.grid(True)
plt.legend()

# Esfuerzo
plt.subplot(2, 1, 2)
plt.plot(timestamps, efforts, color='orange', label='Esfuerzo PID')
plt.xlabel('Tiempo [s]')
plt.ylabel('Esfuerzo')
plt.grid(True)
plt.legend()

plt.suptitle('Evolución del control del mástil extensible')
plt.tight_layout()
plt.show()
