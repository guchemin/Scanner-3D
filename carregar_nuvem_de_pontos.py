import open3d as o3d
import numpy as np
from scipy.ndimage import uniform_filter1d

def colorize_point_cloud(point_cloud):
    points = np.asarray(point_cloud.points)

    # Calcular a distância de cada ponto até a origem (0,0) no plano XY
    distances = np.sqrt(points[:, 0]**2 + points[:, 1]**2)

    # Normalizar as distâncias para o intervalo [0, 1]
    distances_normalized = (distances - distances.min()) / (distances.max() - distances.min())

    # Criar cores com base na distância ao ponto (0,0)
    colors = np.zeros((points.shape[0], 3))
    colors[:, 0] = distances_normalized * 0.75  # Red channel
    colors[:, 1] = distances_normalized * 0.75  # Green channel
    colors[:, 2] = distances_normalized * 0.75  # Blue channel

    # Atribuir cores à nuvem de pontos
    point_cloud.colors = o3d.utility.Vector3dVector(colors)

    return point_cloud
def close_open_slices(point_cloud, threshold, num_interpolation_points= 3):
    points = np.asarray(point_cloud.points)

    # Identificar os limites da nuvem de pontos
    z_min = np.min(points[:, 2])
    z_max = np.max(points[:, 2])

    # Identificar as fatias superior e inferior abertas com base no threshold
    lower_slice = points[points[:, 2] < z_min + threshold]
    upper_slice = points[points[:, 2] > z_max - threshold]

    # Verificar se existem pontos suficientes nas fatias para interpolar
    if lower_slice.size > 0 and upper_slice.size > 0:
        # Para cada ponto na fatia inferior, encontramos o ponto correspondente na fatia superior mais próxima
        interpolated_points = []
        for lower_point in lower_slice:
            # Encontrar o ponto mais próximo na fatia superior
            distances = np.linalg.norm(upper_slice[:, :2] - lower_point[:2], axis=1)
            closest_upper_point = upper_slice[np.argmin(distances)]

            # Interpolar entre o ponto inferior e o superior
            interp_points = np.linspace(lower_point, closest_upper_point, num=num_interpolation_points)
            interpolated_points.append(interp_points)

        interpolated_points = np.vstack(interpolated_points)

        # Combinar os pontos interpolados com a nuvem original
        closed_points = np.vstack((points, interpolated_points))

        closed_cloud = o3d.geometry.PointCloud()
        closed_cloud.points = o3d.utility.Vector3dVector(closed_points)

        return closed_cloud
    else:
        print("Não há fatias abertas suficientes para interpolação.")
        return point_cloud


def moving_average_filter(point_cloud, size):
    points = np.asarray(point_cloud.points)
    # Aplicar filtro de média móvel em cada dimensão
    smoothed_points = np.copy(points)
    for i in range(points.shape[1]):  # Para cada dimensão (x, y, z)
        smoothed_points[:, i] = uniform_filter1d(points[:, i], size=size, mode='reflect')
    # Criar uma nova nuvem de pontos suavizada
    smoothed_cloud = o3d.geometry.PointCloud()
    smoothed_cloud.points = o3d.utility.Vector3dVector(smoothed_points)
    return smoothed_cloud


def compute_point_cloud_volume(point_cloud, voxel_size):
    # Criação de uma grade de voxels
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(point_cloud, voxel_size)

    # Número de voxels ocupados
    occupied_voxels = len(voxel_grid.get_voxels())

    # Volume de um voxel
    voxel_volume = voxel_size ** 3

    # Estimar o volume total
    total_volume = occupied_voxels * voxel_volume
    return total_volume

# Carregar a nuvem de pontos
point_cloud = o3d.io.read_point_cloud("nuvem_de_pontos_final.ply")

# Remoção de outliers
cl, ind = point_cloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
filtered_cloud = point_cloud.select_by_index(ind)

# Suavização da nuvem de pontos usando filtro de média móvel
smoothed_cloud = moving_average_filter(filtered_cloud, size=10)

# Visualizar a nuvem de pontos suavizada
# o3d.visualization.draw_geometries([smoothed_cloud], point_show_normal=True)

# Fechar as extremidades abertas
closed_cloud = close_open_slices(smoothed_cloud, threshold=3)

# Visualizar a nuvem de pontos fechada
# o3d.visualization.draw_geometries([closed_cloud], point_show_normal=True)

# Colorir a nuvem de pontos com base nas coordenadas X e Y
colored_cloud = colorize_point_cloud(closed_cloud)

# Visualizar a nuvem de pontos colorida
o3d.visualization.draw_geometries([colored_cloud], point_show_normal=True)




voxel_size = 1.65  # Ajustar o tamanho do voxel conforme necessário
volume = compute_point_cloud_volume(colored_cloud, voxel_size)

print(f"Volume estimado da nuvem de pontos: {volume}")