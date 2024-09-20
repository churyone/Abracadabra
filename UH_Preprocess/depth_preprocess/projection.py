import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import os
import cv2
from preprocessing import preprocess, classify_plane_obstacle, clustering  # 비디오 생성을 위함
from sklearn.cluster import DBSCAN
from scipy.spatial import ConvexHull

def projecting(obstacle_cloud, plane_model):
    # 평면 계수를 추출합니다.
    a, b, c, d = plane_model

    # 평면의 법선 벡터를 정의합니다.
    plane_normal = np.array([a, b, c])

    # 장애물 클라우드의 각 점을 평면에 정사영합니다.
    projected_points = []
    for point in obstacle_cloud.points:
        point = np.array(point)
        # 평면의 법선을 따라 점에서 평면까지의 거리를 계산합니다.
        distance = (np.dot(plane_normal, point) + d) / np.linalg.norm(plane_normal)
        # 점을 평면에 정사영합니다.
        projected_point = point - distance * plane_normal
        projected_points.append(projected_point)  # 3D 포인트를 그대로 저장합니다.

    # 정사영된 점들을 numpy 배열로 변환합니다.
    projected_points = np.array(projected_points)

    return projected_points

def visualize_projecting(projected_points):
    # 3D 포인트 클라우드 생성
    projected_cloud = o3d.geometry.PointCloud()
    projected_cloud.points = o3d.utility.Vector3dVector(projected_points)
    
    # 시각화 색상을 설정합니다. (평면에 정사영된 포인트들은 녹색으로 표시)
    projected_cloud.paint_uniform_color([0.0, 1.0, 0.0])

    # 시각화
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    # 투영된 포인트 클라우드를 추가합니다.
    vis.add_geometry(projected_cloud)

    # 좌표축 추가
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
    vis.add_geometry(coordinate_frame)

    vis.run()
    vis.destroy_window()

def project_to_2d(obstacle_cloud, plane_model):
    # 평면 계수를 추출합니다.
    a, b, c, d = plane_model
    plane_normal = np.array([a, b, c])

    # 로컬 원점을 평면에 투영합니다.
    origin = np.array([0, 0, 0])
    distance_to_plane = (np.dot(plane_normal, origin) + d) / np.linalg.norm(plane_normal)
    local_origin = origin - distance_to_plane * plane_normal

    # (-1, 0, 0) 벡터를 평면에 투영하여 Local X 축을 정의합니다.
    negative_x = np.array([-1, 0, 0])
    distance_to_plane = (np.dot(plane_normal, negative_x) + d) / np.linalg.norm(plane_normal)
    local_x_point = negative_x - distance_to_plane * plane_normal
    local_x = local_x_point - local_origin
    local_x /= np.linalg.norm(local_x)  # 정규화

    # (0, 0, 1) 벡터를 평면에 투영하여 Local Y 축을 정의합니다.
    z_axis = np.array([0, 0, 1])
    distance_to_plane = (np.dot(plane_normal, z_axis) + d) / np.linalg.norm(plane_normal)
    local_y_point = z_axis - distance_to_plane * plane_normal
    local_y = local_y_point - local_origin
    local_y /= np.linalg.norm(local_y)  # 정규화

    # 장애물 클라우드의 각 점을 평면에 정사영하고 로컬 좌표계로 변환합니다.
    projected_points = []
    for point in obstacle_cloud.points:
        point = np.array(point)
        distance_to_plane = (np.dot(plane_normal, point) + d) / np.linalg.norm(plane_normal)
        projected_point = point - distance_to_plane * plane_normal
        
        # 로컬 좌표계로 변환
        local_x_value = np.dot(projected_point - local_origin, local_x)
        local_y_value = np.dot(projected_point - local_origin, local_y)
        projected_points.append([local_x_value, local_y_value])

    # 정사영된 점들을 numpy 배열로 변환합니다.
    projected_points = np.array(projected_points)

    return projected_points

def cluster_and_fit_shapes_2d(projected_points, eps=0.05, min_samples=5):
    """
    2D 포인트 클라우드를 클러스터링하고 각 클러스터에 대해 기본 도형을 맞추는 함수입니다.
    
    Parameters:
    - projected_points: 2D로 투영된 포인트 클라우드 (numpy 배열)
    - eps: DBSCAN 클러스터링 반경 파라미터
    - min_samples: 클러스터를 형성하기 위한 최소 포인트 수
    
    Returns:
    - clusters: 각 클러스터에 속하는 포인트들의 리스트
    - shapes: 각 클러스터의 포인트를 둘러싸는 도형 (Convex Hull) 리스트
    """
    db = DBSCAN(eps=eps, min_samples=min_samples).fit(projected_points)
    labels = db.labels_
    unique_labels = set(labels)

    clusters = []
    shapes = []

    for k in unique_labels:
        if k == -1:
            continue  # 노이즈 포인트는 -1로 라벨링 됨

        class_member_mask = (labels == k)
        cluster = projected_points[class_member_mask]
        clusters.append(cluster)

        if len(cluster) >= 3:
            hull = ConvexHull(cluster)
            hull_points = cluster[hull.vertices]
            shapes.append(hull_points)
        else:
            shapes.append(cluster)  # 포인트 수가 적으면 도형 대신 포인트 그대로 사용

    return clusters, shapes

def visualize_shapes_2d(projected_points, shapes):
    """
    2D로 투영된 포인트 클라우드와 각 클러스터에 맞춘 도형들을 시각화하는 함수입니다.
    
    Parameters:
    - projected_points: 2D로 투영된 포인트 클라우드 (numpy 배열)
    - shapes: 각 클러스터의 포인트를 둘러싸는 도형 (Convex Hull 또는 Bounding Box) 리스트
    """
    plt.figure(figsize=(8, 8))
    plt.scatter(projected_points[:, 0], projected_points[:, 1], c='gray', marker='o', s=5, alpha=0.5, label='Points')

    # 클러스터의 도형들을 시각화
    for shape in shapes:
        plt.plot(np.append(shape[:, 0], shape[0, 0]), np.append(shape[:, 1], shape[0, 1]), 'r-', lw=2)

    plt.title("2D Projection with Clustered Shapes")
    plt.xlabel("Local X")
    plt.ylabel("Local Y")
    plt.grid(True)
    plt.legend()
    plt.show()

def visualize_2d_projection(projected_points):
    """
    장애물의 2D 투영을 시각화합니다.
    
    Parameters:
    - projected_points: np.array, 2D로 투영된 장애물 포인트 배열 (x, y)
    """
    plt.figure(figsize=(8, 8))
    plt.scatter(projected_points[:, 0], projected_points[:, 1], c='blue', marker='o', s=1, label='Obstacles')
    plt.title("2D Projection of Obstacles")
    plt.xlabel("Local X")
    plt.ylabel("Local Y")
    plt.grid(True)
    plt.legend()
    plt.show()

def visualize_2d_video(image_directory, fx, fy, cx, cy, depth_scale_factor, voxel_size):
    depth_images = sorted(os.listdir(image_directory))

    for filename in depth_images:
        if filename.endswith('.png'):
            depth_image_path = os.path.join(image_directory, filename)
            depth_image = cv2.imread(depth_image_path, cv2.IMREAD_UNCHANGED)

            if depth_image is not None:
                # 1. 포인트 클라우드 생성
                point_cloud = preprocess(depth_image_path, fx, fy, cx, cy, depth_scale_factor, voxel_size)
                
                # 2. 평면 탐지 및 장애물 분류
                inlier_cloud, obstacle_cloud, plane_model = classify_plane_obstacle(point_cloud)
                
                # 3. 장애물 클러스터링
                obstacle_cloud = clustering(obstacle_cloud)

                # 4. 장애물의 2D 투영
                projected_points_2d = project_to_2d(obstacle_cloud, plane_model)
                
                # 5. 클러스터링 및 도형 생성
                clusters, shapes = cluster_and_fit_shapes_2d(projected_points_2d)
                
                # 6. 2D 시각화
                visualize_shapes_2d(projected_points_2d, shapes)

            else:
                print(f"{filename} 파일을 로드할 수 없습니다.")
