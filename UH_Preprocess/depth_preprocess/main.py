from preprocessing import preprocess, classify_plane_obstacle, visualize, visualization
from projection import project_to_2d, cluster_and_fit_shapes_2d, visualize_shapes_2d
from path_decision import visualize_optimal_direction  # 최적 방향 시각화 함수 사용

def main(depth_image_path, image_directory, fx, fy, cx, cy, depth_scale_factor, voxel_size):
    # 1. 깊이 이미지 전처리 및 포인트 클라우드 생성
    point_cloud = preprocess(depth_image_path, fx, fy, cx, cy, depth_scale_factor, voxel_size)
    
    # 2. RANSAC 평면 탐지 및 장애물 분류
    inlier_cloud, obstacle_cloud, plane_model = classify_plane_obstacle(point_cloud)
    
    # 3. 3D 시각화
    visualize(point_cloud)
    visualization(inlier_cloud, obstacle_cloud)

    # 4. 평면 투영 2D
    projected_points = project_to_2d(obstacle_cloud, plane_model)
    
    # 5. 2D 투영된 포인트 클라우드 클러스터링 및 도형 생성
    clusters, shapes = cluster_and_fit_shapes_2d(projected_points)
    visualize_shapes_2d(projected_points, shapes)
    # 6. 최적 방향 계산 및 시각화
    visualize_optimal_direction(shapes)

if __name__ == "__main__":
    # 카메라 파라미터 및 이미지 경로 설정
    depth_image_path = 'C:/Users/User/Desktop/cap_dataset/rgbd_dataset_freiburg1_floor/rgbd_dataset_freiburg1_floor/depth/1305033573.015430.png'
    image_directory = 'C:/Users/User/Desktop/cap_dataset/rgbd_dataset_freiburg1_floor/rgbd_dataset_freiburg1_floor/depth/'

    fx, fy = 517.3, 516.5  # 카메라의 초점 거리
    cx, cy = 318.6, 255.3  # 카메라의 주점 (Principal Point)
    depth_scale_factor = 5000.0  # 깊이 값 스케일링 팩터
    voxel_size = 0.01  # 샘플링 간격 (0.01m)
    
    main(depth_image_path, image_directory, fx, fy, cx, cy, depth_scale_factor, voxel_size)
