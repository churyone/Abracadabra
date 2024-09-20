import open3d as o3d
import numpy as np
import cv2
import os
import time

# 이미지가 있는 디렉토리 경로
image_directory = 'C:/Users/User/Desktop/cap_dataset/rgbd_dataset_freiburg1_floor/rgbd_dataset_freiburg1_floor/depth/'

# 카메라 파라미터 설정
fx, fy = 517.3, 516.5  # 초점 거리
cx, cy = 318.6, 255.3  # 주점 (Principal point)
depth_scale_factor = 5000.0  # 깊이 스케일링 팩터
voxel_size = 0.01  # 샘플링 간격 (0.01m) 즉 1cm 단위로 포인트 클라우드를 여과하여 얻어옴.

# 디렉토리 내 파일 목록 불러오기
depth_images = sorted(os.listdir(image_directory))

# Open3D 시각화 설정
vis = o3d.visualization.Visualizer()
vis.create_window()

# PointCloud 객체 생성
pcd = o3d.geometry.PointCloud()

for filename in depth_images:
    if filename.endswith('.png'):  # 확장자가 .png인 파일만 처리
        depth_image_path = os.path.join(image_directory, filename)
        depth_image = cv2.imread(depth_image_path, cv2.IMREAD_UNCHANGED)

        if depth_image is not None:
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

            # Open3D 포인트 클라우드로 변환
            pcd.points = o3d.utility.Vector3dVector(points)
            pcd.colors = o3d.utility.Vector3dVector(colors)

            # RANSAC 평면 피팅을 수행하여 지면 평면 추정
            plane_model, inliers = pcd.segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=100)
            [a, b, c, d] = plane_model
            print(f"파일: {filename}, 평면 방정식: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

            # 평면 위의 포인트들 추출 (inliers)
            inlier_cloud = pcd.select_by_index(inliers)
            inlier_cloud.paint_uniform_color([0.6, 0.6, 0.6])  # 지면 평면 포인트들을 회색으로 표시

            # 평면에 속하지 않는 포인트들 추출 (outliers)
            outlier_cloud = pcd.select_by_index(inliers, invert=True)

            # 장애물을 식별하기 위한 높이 임계값 정의
            distance_from_plane = np.abs(np.asarray(outlier_cloud.points) @ np.array([a, b, c]) + d) / np.linalg.norm([a, b, c])
            height_threshold = 0.03  # 장애물을 위한 높이 임계값 설정
            obstacle_indices = np.where(distance_from_plane > height_threshold)[0]
            obstacle_cloud = outlier_cloud.select_by_index(obstacle_indices)

            # 시각화 업데이트
            vis.clear_geometries()
            vis.add_geometry(inlier_cloud)
            vis.add_geometry(obstacle_cloud)
            vis.poll_events()
            vis.update_renderer()

            # 프레임 간 지연 (영상처럼 송출되도록)
            time.sleep(0.1)
        else:
            print(f"{filename} 파일을 로드할 수 없습니다.")

# 시각화 창 닫기
vis.destroy_window()

'''
import open3d as o3d
import numpy as np
import cv2
import os
import time

# 이미지가 있는 디렉토리 경로
image_directory = 'C:/Users/User/Desktop/cap_dataset/rgbd_dataset_freiburg1_floor/rgbd_dataset_freiburg1_floor/depth/'

# 카메라 파라미터 설정
fx, fy = 517.3, 516.5  # 초점 거리
cx, cy = 318.6, 255.3  # 주점 (Principal point)
depth_scale_factor = 5000.0  # 깊이 스케일링 팩터
voxel_size = 0.02  # Voxel 크기 설정

# 디렉토리 내 파일 목록 불러오기
depth_images = sorted(os.listdir(image_directory))

# Open3D 시각화 설정
vis = o3d.visualization.Visualizer()
vis.create_window()

# PointCloud 객체 생성
pcd = o3d.geometry.PointCloud()

for filename in depth_images:
    if filename.endswith('.png'):  # 확장자가 .png인 파일만 처리
        depth_image_path = os.path.join(image_directory, filename)
        depth_image = cv2.imread(depth_image_path, cv2.IMREAD_UNCHANGED)

        if depth_image is not None:
            height, width = depth_image.shape

            points = []
            colors = []

            min_depth = np.min(depth_image[depth_image > 0]) / depth_scale_factor  # 최소 깊이 값
            max_depth = np.max(depth_image) / depth_scale_factor  # 최대 깊이 값

            for v in range(height):
                for u in range(width):
                    Z = depth_image[v, u] / depth_scale_factor
                    if Z > 0:
                        X = (u - cx) * Z / fx
                        Y = (v - cy) * Z / fy
                        points.append([X, Y, Z])

                        # Z 값을 기반으로 색상 지정 (빨간색은 가까운 거리, 파란색은 먼 거리)
                        normalized_depth = (Z - min_depth) / (max_depth - min_depth)
                        color = [1.0 - normalized_depth, 0, normalized_depth]  # 빨간색에서 파란색으로 변화
                        colors.append(color)

            # Open3D 포인트 클라우드로 변환
            point_cloud = o3d.geometry.PointCloud()
            point_cloud.points = o3d.utility.Vector3dVector(points)
            point_cloud.colors = o3d.utility.Vector3dVector(colors)

            # Voxel Downsampling 적용
            point_cloud = point_cloud.voxel_down_sample(voxel_size=voxel_size)

            # RANSAC 평면 피팅을 수행하여 지면 평면 추정
            plane_model, inliers = point_cloud.segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=100)
            [a, b, c, d] = plane_model
            print(f"파일: {filename}, 평면 방정식: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

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

            # 시각화 업데이트
            vis.clear_geometries()
            vis.add_geometry(inlier_cloud)
            vis.add_geometry(obstacle_cloud)
            vis.poll_events()
            vis.update_renderer()

            # 프레임 간 지연 (영상처럼 송출되도록)
            time.sleep(0.1)
        else:
            print(f"{filename} 파일을 로드할 수 없습니다.")

# 시각화 창 닫기
vis.destroy_window()
'''