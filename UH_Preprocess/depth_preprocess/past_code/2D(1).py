import open3d as o3d
import numpy as np
import cv2
import matplotlib.pyplot as plt

def load_depth_image(depth_image_path):
    """깊이 이미지를 로드합니다."""
    return cv2.imread(depth_image_path, cv2.IMREAD_UNCHANGED)

def compute_point_cloud(depth_image, fx, fy, cx, cy, depth_scale_factor, voxel_size):
    """깊이 이미지로부터 포인트 클라우드를 계산합니다."""
    height, width = depth_image.shape
    points = []
    colors = []
    
    min_depth = np.min(depth_image[depth_image > 0]) / depth_scale_factor
    max_depth = np.max(depth_image) / depth_scale_factor
    
    for v in range(0, height, int(voxel_size * fy)):
        for u in range(0, width, int(voxel_size * fx)):
            Z = depth_image[v, u] / depth_scale_factor
            if Z > 0:
                X = (u - cx) * Z / fx
                Y = (v - cy) * Z / fy
                points.append([X, Y, Z])
                
                normalized_depth = (Z - min_depth) / (max_depth - min_depth)
                color = [1.0 - normalized_depth, 0, normalized_depth]
                colors.append(color)
    
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    point_cloud.colors = o3d.utility.Vector3dVector(colors)
    
    return point_cloud

def extract_obstacles(point_cloud, plane_model, distance_threshold=0.03):
    """포인트 클라우드에서 평면을 제거하고 장애물 포인트를 추출합니다."""
    [a, b, c, d] = plane_model
    inliers = point_cloud.segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=100)[1]
    outlier_cloud = point_cloud.select_by_index(inliers, invert=True)
    
    distance_from_plane = np.abs(np.asarray(outlier_cloud.points) @ np.array([a, b, c]) + d) / np.linalg.norm([a, b, c])
    obstacle_indices = np.where(distance_from_plane > distance_threshold)[0]
    
    return outlier_cloud.select_by_index(obstacle_indices)

def project_points_to_plane(points, plane_model):
    """포인트들을 평면에 투영하여 2D 평면상의 좌표로 변환합니다."""
    [a, b, c, d] = plane_model
    normal = np.array([a, b, c])
    normal /= np.linalg.norm(normal)
    projection_matrix = np.eye(3) - np.outer(normal, normal)
    
    projected_points = []
    for point in np.asarray(points):
        projected_point = projection_matrix @ point
        projected_points.append(projected_point[:2])  # x, y 좌표만 사용 (2D 평면)
    
    return np.array(projected_points)

def create_2d_bounding_boxes(projected_points, cluster_labels):
    """2D 평면상에서 클러스터에 대한 바운딩 박스를 생성합니다."""
    bounding_boxes = []
    for label in np.unique(cluster_labels):
        if label == -1:  # noise points
            continue
        cluster_points = projected_points[cluster_labels == label]
        if len(cluster_points) > 0:
            min_point = cluster_points.min(axis=0)
            max_point = cluster_points.max(axis=0)
            bbox = np.array([
                [min_point[0], min_point[1]],
                [max_point[0], min_point[1]],
                [max_point[0], max_point[1]],
                [min_point[0], max_point[1]],
            ])
            bounding_boxes.append(bbox)
    
    return bounding_boxes

def visualize_2d_bounding_boxes(bounding_boxes, camera_position, camera_direction):
    """2D 바운딩 박스를 시각화하고 카메라 시점을 화살표로 표시합니다."""
    plt.figure(figsize=(10, 10))
    
    # 바운딩 박스 시각화
    for bbox in bounding_boxes:
        plt.plot([bbox[i, 0] for i in range(4)] + [bbox[0, 0]], 
                 [bbox[i, 1] for i in range(4)] + [bbox[0, 1]], 
                 color='red')
    
    # 카메라 시점 화살표 시각화
    plt.arrow(camera_position[0], camera_position[1],
              camera_direction[0], camera_direction[1],
              head_width=0.05, head_length=0.1, fc='blue', ec='blue')
    
    plt.title('2D Projection of Bounding Boxes on the Ground Plane')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis('equal')
    plt.grid(True)
    plt.show()

def main():
    # 설정 값들
    depth_image_path = 'C:/Users/User/Desktop/cap_dataset/rgbd_dataset_freiburg1_floor/rgbd_dataset_freiburg1_floor/depth/1305033532.903424.png'
    fx, fy = 517.3, 516.5
    cx, cy = 318.6, 255.3
    depth_scale_factor = 5000.0
    voxel_size = 0.01
    
    # 깊이 이미지 로드 및 포인트 클라우드 생성
    depth_image = load_depth_image(depth_image_path)
    if depth_image is None:
        print("깊이 이미지 로드 실패.")
        return
    
    point_cloud = compute_point_cloud(depth_image, fx, fy, cx, cy, depth_scale_factor, voxel_size)
    
    # 평면 탐지 및 제거
    plane_model, inliers = point_cloud.segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=100)
    print(f"평면 방정식: {plane_model[0]:.2f}x + {plane_model[1]:.2f}y + {plane_model[2]:.2f}z + {plane_model[3]:.2f} = 0")
    
    # 장애물 추출
    obstacle_cloud = extract_obstacles(point_cloud, plane_model)
    
    # 장애물 군집화
    cluster_labels = np.array(obstacle_cloud.cluster_dbscan(eps=0.05, min_points=10))
    max_label = cluster_labels.max()
    print(f"총 군집 수: {max_label + 1}")
    
    # 3D 포인트를 평면에 투영하여 2D 평면상 좌표로 변환
    projected_points = project_points_to_plane(obstacle_cloud.points, plane_model)
    
    # 2D 바운딩 박스 생성
    bounding_boxes = create_2d_bounding_boxes(projected_points, cluster_labels)
    
    # 카메라 위치와 방향 설정
    camera_position = np.array([0.0, 0.0])  # 원점
    camera_direction = np.array([1.0, 0.0])  # x축 방향을 가리키도록 설정
    
    # 2D 바운딩 박스 및 카메라 시점 화살표 시각화
    visualize_2d_bounding_boxes(bounding_boxes, camera_position, camera_direction)

if __name__ == "__main__":
    main()
