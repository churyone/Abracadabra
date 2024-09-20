import open3d as o3d
import numpy as np
import cv2
import matplotlib.pyplot as plt
import os
import time

def preprocess(depth_image_path, fx, fy, cx, cy, depth_scale_factor, voxel_size):
    # 깊이 이미지 로드
    depth_image = cv2.imread(depth_image_path, cv2.IMREAD_UNCHANGED)
    if depth_image is None:
        raise ValueError("Failed to load depth image.")

    height, width = depth_image.shape
    points = []
    colors = []
    
    # 깊이 값의 최소/최대 값 계산
    min_depth = np.min(depth_image[depth_image > 0]) / depth_scale_factor
    max_depth = np.max(depth_image) / depth_scale_factor
    
    # 깊이 이미지를 다운샘플링하여 포인트 클라우드 생성
    for v in range(0, height, int(voxel_size * fy)):
        for u in range(0, width, int(voxel_size * fx)):
            Z = depth_image[v, u] / depth_scale_factor
            if Z > 0:  # 깊이 값이 0 이상인 경우만 처리
                X = (u - cx) * -Z / fx  # 픽셀 좌표를 카메라 좌표계로 변환
                Y = (v - cy) * -Z / fy
                points.append([X, Y, Z])
                
                # 깊이 값을 기반으로 색상 지정 (빨간색에서 파란색으로 변화)
                normalized_depth = (Z - min_depth) / (max_depth - min_depth)
                color = [1.0 - normalized_depth, 0, normalized_depth]
                colors.append(color)
    
    # Open3D 포인트 클라우드 객체 생성
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    point_cloud.colors = o3d.utility.Vector3dVector(colors)
    
    return point_cloud

def classify_plane_obstacle(point_cloud, distance_threshold=0.02, ransac_n=3, num_iterations=100, height_threshold=0.03):
    # 평면 탐지: RANSAC 알고리즘 사용
    plane_model, inliers = point_cloud.segment_plane(distance_threshold=distance_threshold, ransac_n=ransac_n, num_iterations=num_iterations)
    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
    
    # 탐지된 평면에 속하는 점들
    inlier_cloud = point_cloud.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([0.6, 0.6, 0.6])  # 평면을 회색으로 표시
    
    # 평면에 속하지 않는 점들 (장애물 가능성 있음)
    outlier_cloud = point_cloud.select_by_index(inliers, invert=True)
    
    # 평면과의 거리 계산하여 장애물 분류
    distance_from_plane = np.abs(np.asarray(outlier_cloud.points) @ np.array([a, b, c]) + d) / np.linalg.norm([a, b, c])
    obstacle_indices = np.where(distance_from_plane > height_threshold)[0]
    obstacle_cloud = outlier_cloud.select_by_index(obstacle_indices)
    
    return inlier_cloud, obstacle_cloud

def clustering(obstacle_cloud, eps=0.05, min_points=10):
    # 장애물 클라우드에서 유클리디안 클러스터링 수행
    cluster_labels = np.array(obstacle_cloud.cluster_dbscan(eps=eps, min_points=min_points))
    max_label = cluster_labels.max()
    print(f"Number of clusters: {max_label + 1}")
    
    # 각 클러스터에 색상 할당
    colors = plt.get_cmap("tab20")(cluster_labels / (max_label if max_label > 0 else 1))
    obstacle_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
    
    return obstacle_cloud

def visualization(inlier_cloud, obstacle_cloud):
    # 카메라 중심을 표시하는 화살표 생성
    camera_center = np.array([0.0, 0.0, 0.0])
    camera_color = [1.0, 0.0, 1.0]
    
    camera_arrow = o3d.geometry.TriangleMesh.create_arrow(cylinder_radius=0.01, cone_radius=0.02, cylinder_height=0.1, cone_height=0.04)
    camera_arrow.paint_uniform_color(camera_color)
    camera_arrow.translate(camera_center)
    
    # 좌표계 표시
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])

    # 시각화 윈도우 생성
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(inlier_cloud)
    vis.add_geometry(obstacle_cloud)
    vis.add_geometry(camera_arrow)
    vis.add_geometry(coordinate_frame)
    
    # 카메라 뷰 설정
    ctr = vis.get_view_control()
    ctr.set_front([0.0, 0.0, -1.0])
    ctr.set_lookat([0.0, 0.0, 0.0])
    ctr.set_up([0.0, 1.0, 0.0])
    ctr.set_zoom(0.5)
    
    # 시각화 실행
    vis.run()
    vis.destroy_window()

def visualization_video(image_directory, fx, fy, cx, cy, depth_scale_factor, voxel_size):
    # 디렉토리 내 파일 목록 불러오기
    depth_images = sorted(os.listdir(image_directory))

    # Open3D 시각화 설정
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    for filename in depth_images:
        if filename.endswith('.png'):  # 확장자가 .png인 파일만 처리
            depth_image_path = os.path.join(image_directory, filename)
            depth_image = cv2.imread(depth_image_path, cv2.IMREAD_UNCHANGED)

            if depth_image is not None:
                # 포인트 클라우드 생성
                point_cloud = preprocess(depth_image_path, fx, fy, cx, cy, depth_scale_factor, voxel_size)

                # 평면 탐지 및 장애물 분류
                inlier_cloud, obstacle_cloud = classify_plane_obstacle(point_cloud)

                # 시각화 윈도우에 추가
                vis.clear_geometries()
                vis.add_geometry(inlier_cloud)
                vis.add_geometry(obstacle_cloud)
                
                # 카메라 중심과 좌표계를 시각화에 추가
                camera_center = np.array([0.0, 0.0, 0.0])
                camera_color = [1.0, 0.0, 1.0]
                
                camera_arrow = o3d.geometry.TriangleMesh.create_arrow(cylinder_radius=0.01, cone_radius=0.02, cylinder_height=0.1, cone_height=0.04)
                camera_arrow.paint_uniform_color(camera_color)
                camera_arrow.translate(camera_center)
                
                coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
                
                vis.add_geometry(camera_arrow)
                vis.add_geometry(coordinate_frame)

                # 카메라 뷰 설정
                ctr = vis.get_view_control()
                ctr.set_front([0.0, 0.0, -1.0])
                ctr.set_lookat([0.0, 0.0, 0.0])
                ctr.set_up([0.0, 1.0, 0.0])
                ctr.set_zoom(0.5)

                # 시각화 업데이트
                vis.poll_events()
                vis.update_renderer()

                # 프레임 간 지연 (영상처럼 송출되도록)
                time.sleep(0.1)
            else:
                print(f"{filename} 파일을 로드할 수 없습니다.")

    # 시각화 창 닫기
    vis.destroy_window()

def main(depth_image_path, fx, fy, cx, cy, depth_scale_factor, voxel_size):
    # 1. 깊이 이미지 전처리 및 포인트 클라우드 생성
    point_cloud = preprocess(depth_image_path, fx, fy, cx, cy, depth_scale_factor, voxel_size)
    
    # 2. RANSAC 평면 탐지 및 장애물 분류
    inlier_cloud, obstacle_cloud = classify_plane_obstacle(point_cloud)
    
    # 3. 장애물에 대한 유클리디안 클러스터링 수행
    obstacle_cloud = clustering(obstacle_cloud)
    
    # 4. 시각화
    visualization(inlier_cloud, obstacle_cloud)
    #visualization_video(image_directory, fx, fy, cx, cy, depth_scale_factor, voxel_size)


if __name__ == "__main__":
    # 카메라 파라미터 및 이미지 경로 설정
    depth_image_path = 'C:/Users/User/Desktop/cap_dataset/rgbd_dataset_freiburg1_floor/rgbd_dataset_freiburg1_floor/depth/1305033532.903424.png'
    image_directory = 'C:/Users/User/Desktop/cap_dataset/rgbd_dataset_freiburg1_floor/rgbd_dataset_freiburg1_floor/depth/'

    fx, fy = 517.3, 516.5  # 카메라의 초점 거리
    cx, cy = 318.6, 255.3  # 카메라의 주점 (Principal Point)
    depth_scale_factor = 5000.0  # 깊이 값 스케일링 팩터
    voxel_size = 0.01  # 샘플링 간격 (0.01m)
    
    main(depth_image_path, fx, fy, cx, cy, depth_scale_factor, voxel_size)
