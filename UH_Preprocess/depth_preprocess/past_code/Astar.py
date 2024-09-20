import numpy as np
import open3d as o3d
import cv2
import matplotlib.pyplot as plt
from queue import PriorityQueue

def load_depth_image(depth_image_path, fx, fy, cx, cy, depth_scale_factor, voxel_size):
    depth_image = cv2.imread(depth_image_path, cv2.IMREAD_UNCHANGED)

    if depth_image is None:
        print("깊이 이미지 로드 실패.")
        return None, None

    height, width = depth_image.shape
    points = []
    colors = []

    min_depth = np.min(depth_image[depth_image > 0]) / depth_scale_factor  # 최소 깊이 값
    max_depth = np.max(depth_image) / depth_scale_factor  # 최대 깊이 값

    for v in range(0, height, int(voxel_size * fy)):  # y축 방향으로 샘플링 간격 적용
        for u in range(0, width, int(voxel_size * fx)):  # x축 방향으로 샘플링 간격 적용
            Z = depth_image[v, u] / depth_scale_factor
            if Z > 0:
                X = (u - cx) * Z / fx
                Y = (v - cy) * Z / fy
                points.append([X, Y, Z])

                # Z 값을 기반으로 색상 지정 (빨간색은 가까운 거리, 파란색은 먼 거리)
                normalized_depth = (Z - min_depth) / (max_depth - min_depth)
                color = [1.0 - normalized_depth, 0, normalized_depth]  # 빨간색에서 파란색으로 변화
                colors.append(color)

    return points, colors

def process_point_cloud(points, colors):
    # Open3D 포인트 클라우드로 변환
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    point_cloud.colors = o3d.utility.Vector3dVector(colors)

    # 평면 탐지 및 제거
    plane_model, inliers = point_cloud.segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=100)
    [a, b, c, d] = plane_model
    print(f"평면 방정식: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

    # 평면 위의 포인트들 추출 (inliers)
    inlier_cloud = point_cloud.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([0.6, 0.6, 0.6])  # 지면 평면 포인트들을 회색으로 표시

    # 평면에 속하지 않는 포인트들 추출 (outliers)
    outlier_cloud = point_cloud.select_by_index(inliers, invert=True)

    # 장애물을 식별하기 위한 높이 임계값 정의
    distance_from_plane = np.abs(np.asarray(outlier_cloud.points) @ np.array([a, b, c]) + d) / np.linalg.norm([a, b, c])
    height_threshold = 0.03  # 장애물을 위한 높이 임계값 설정
    obstacle_indices = np.where(distance_from_plane > height_threshold)[0]
    obstacle_cloud = outlier_cloud.select_by_index(obstacle_indices)

    # 장애물 군집화 (Euclidean 클러스터링)
    cluster_labels = np.array(obstacle_cloud.cluster_dbscan(eps=0.05, min_points=10))

    # 군집화된 결과를 시각화하기 위한 색상 팔레트 생성
    max_label = cluster_labels.max()
    print(f"총 군집 수: {max_label + 1}")

    colors = plt.get_cmap("tab20")(cluster_labels / (max_label if max_label > 0 else 1))
    obstacle_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])  # 클러스터에 색상 할당

    return inlier_cloud, obstacle_cloud

def point_cloud_to_2d_map(point_cloud, resolution, height_threshold):
    # 3D 포인트 클라우드를 2D 맵으로 변환
    points = np.asarray(point_cloud.points)
    
    # X, Z 평면으로 투영
    x = points[:, 0]
    z = points[:, 2]
    y = points[:, 1]  # Y는 높이 정보
    
    # 높이 기준으로 필터링 (장애물로 간주)
    obstacle_indices = y > height_threshold
    x = x[obstacle_indices]
    z = z[obstacle_indices]

    # 빈 배열 확인
    if x.size == 0 or z.size == 0:
        print("Height threshold is too high or no obstacles detected. No points available.")
        return None, None, None
    
    # 2D 그리드로 변환
    x_min, x_max = np.min(x), np.max(x)
    z_min, z_max = np.min(z), np.max(z)
    
    grid_x = np.round((x - x_min) / resolution).astype(int)
    grid_z = np.round((z - z_min) / resolution).astype(int)
    
    grid_width = grid_x.max() + 1
    grid_height = grid_z.max() + 1
    
    # 2D 맵 생성
    grid_map = np.zeros((grid_width, grid_height), dtype=np.uint8)
    grid_map[grid_x, grid_z] = 1  # 장애물 표시
    
    return grid_map, (x_min, z_min), resolution

def a_star(grid_map, start, goal):
    # A* 알고리즘 구현
    def heuristic(a, b):
        return np.linalg.norm(np.array(a) - np.array(b))
    
    open_set = PriorityQueue()
    open_set.put((0, start))
    
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    
    while not open_set.empty():
        _, current = open_set.get()
        
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path
        
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            tentative_g_score = g_score[current] + 1
            
            if 0 <= neighbor[0] < grid_map.shape[0] and 0 <= neighbor[1] < grid_map.shape[1]:
                if grid_map[neighbor[0], neighbor[1]] == 1:
                    continue
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    open_set.put((f_score[neighbor], neighbor))
    
    return None  # 경로 없음

def plot_path(grid_map, path):
    plt.imshow(grid_map.T, cmap='gray')
    if path:
        path_x, path_y = zip(*path)
        plt.plot(path_x, path_y, 'r')
    plt.show()

def visualize_point_cloud(inlier_cloud, obstacle_cloud):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    
    vis.add_geometry(inlier_cloud)
    vis.add_geometry(obstacle_cloud)
    
    vis.run()
    vis.destroy_window()

def main():
    # Load the point cloud
    depth_image_path = 'C:/Users/User/Desktop/cap_dataset/rgbd_dataset_freiburg1_floor/rgbd_dataset_freiburg1_floor/depth/1305033532.903424.png'
    fx, fy = 517.3, 516.5  # 초점 거리
    cx, cy = 318.6, 255.3  # 주점 (Principal point)
    depth_scale_factor = 5000.0  # 깊이 스케일링 팩터
    voxel_size = 0.01  # 샘플링 간격 (0.01m)
    
    points, colors = load_depth_image(depth_image_path, fx, fy, cx, cy, depth_scale_factor, voxel_size)
    if points is not None and colors is not None:
        inlier_cloud, obstacle_cloud = process_point_cloud(points, colors)
        
        # 3D 맵 시각화
        visualize_point_cloud(inlier_cloud, obstacle_cloud)
        
        # Convert point cloud to 2D map
        resolution = 0.01  # 2D 맵의 해상도
        height_threshold = 0.02  # 장애물로 간주할 높이
        grid_map, origin, resolution = point_cloud_to_2d_map(obstacle_cloud, resolution, height_threshold)
        
        if grid_map is None:
            print("2D 맵 생성 실패")
            return
        
        # Define start and goal points on the grid
        start = (10, 10)
        goal = (grid_map.shape[0] - 10, grid_map.shape[1] - 10)
        
        # Run A* algorithm
        path = a_star(grid_map, start, goal)
        
        # Plot the path on the grid map
        plot_path(grid_map, path)

if __name__ == "__main__":
    main()
