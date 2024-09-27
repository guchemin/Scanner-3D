import serial
import time
import numpy as np
import open3d as o3d

# Configura a porta serial (ajuste o nome da porta conforme necessário)
ser = serial.Serial("/dev/ttyACM0",
                    9600)  # No Windows, pode ser 'COM3', 'COM4', etc. No Linux, '/dev/ttyUSB0', '/dev/ttyACM0', etc.

# Aguarda um pouco para garantir que a comunicação serial esteja configurada
time.sleep(2)

altura = 0.0  # Altura inicial
angulo = 0.0  # Ângulo inicial
incremento_angulo = 2.0 * np.pi / 211  # Incremento de ângulo em radianos
incremento_altura = 0.4  # Incremento de altura após cada ciclo em cm

pontos = []  # Lista para armazenar os pontos da nuvem
area_atual = 0
area_anterior = 0
volume_total = 0
primeira_dist = 0
dist_atual = 0
dist_anterior = 0
angulo = 2.0 * np.pi / 211  # constante - em radianos
seno = np.sin(angulo)


def median_filter(data, window_size=5, threshold=0.3):
    filtered_data = []
    n = len(data)
    for i in range(0, n, window_size):
        group = data[i:i + window_size]
        if len(group) < window_size:
            filtered_data.extend(group)
            continue
        median = np.median(group)
        for val in group:
            if abs(val - median) > (threshold * median):
                filtered_data.append(median)
            else:
                filtered_data.append(val)
    return filtered_data

def salver(pontos):
    pontos_array = np.array(pontos)
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(pontos_array)

    if len(pontos) == 0:
        print("Nenhum ponto na nuvem de pontos, nada para salvar.")
    else:
        # Tenta salvar a nuvem de pontos em um arquivo PLY
        try:
            o3d.io.write_point_cloud("nuvem_de_pontos_final.ply", point_cloud)
            print("Nuvem de pontos salva com sucesso em 'nuvem_de_pontos_final.ply'.")
        except Exception as e:
            print(f"Erro ao salvar a nuvem de pontos: {e}")


def visualize_point_cloud(points, volume, display_time=22):
    """Cria e exibe a nuvem de pontos usando Open3D por um tempo determinado, com o volume no título da janela."""
    pontos_array = np.array(points)
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(pontos_array)

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=f"Volume atual: {volume:.2f} cm³", width=960, height=540, left=960, top=0)
    vis.add_geometry(point_cloud)

    vis.poll_events()
    vis.update_renderer()

    start_time = time.time()
    while time.time() - start_time < display_time:
        vis.poll_events()
        vis.update_renderer()

    vis.destroy_window()



# No loop principal
try:
    sensor_values = []
    j = 0.0
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            sensor_value = float(line)
            if sensor_value != -1 and sensor_value < 0:
                sensor_value = 0.0
            if sensor_value > 10:
                sensor_value = 10.0

            print(f"Sensor value: {sensor_value}")

            if sensor_value == -1:
                if len(sensor_values) > 1:
                    sensor_values = median_filter(sensor_values)  # Aplica o filtro de mediana

                for value in sensor_values:
                    x = value * np.cos(angulo)
                    y = value * np.sin(angulo)
                    z = altura
                    pontos.append([x, y, z])

                    dist_anterior = dist_atual
                    dist_atual = value
                    area_atual += (seno * dist_atual * dist_anterior) / 2
                    angulo += incremento_angulo


                if area_anterior != 0 and area_atual != 0:
                    volume_total += (area_atual + area_anterior) / 2 * incremento_altura
                elif area_atual == 0:
                    salver(pontos)
                    break

                print(f"Volume atual (altura {j:.2f}): {volume_total}")
                j += 0.4
                altura += incremento_altura
                area_anterior = area_atual
                area_atual = 0
                angulo = 0  # Reinicia o ângulo para começar um novo ciclo
                sensor_values = []

                # Visualiza a nuvem de pontos atualizada com o volume atual no título
                visualize_point_cloud(pontos, volume_total)

            elif sensor_value == -10:
                salver(pontos)
                break

            else:
                sensor_values.append(sensor_value)

except KeyboardInterrupt:
    salver(pontos)

finally:
    ser.close()
    print("Conexão serial encerrada.")

print(f"Volume total calculado: {volume_total}")
with open('volume.txt', 'w') as file:
    file.write(str(volume_total))
