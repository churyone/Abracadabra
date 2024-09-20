import open3d as o3d
import numpy as np
import cv2
import os
import matplotlib.pyplot as plt
import time

# 이미지가 있는 디렉토리 경로
image_directory = 'C:/Users/User/Desktop/cap_dataset/rgbd_dataset_freiburg1_floor/rgbd_dataset_freiburg1_floor/depth/'
rgb_directory = 'C:/Users/User/Desktop/cap_dataset/rgbd_dataset_freiburg1_floor/rgbd_dataset_freiburg1_floor/rgb/'

# 카메라 파라미터 설정
fx, fy = 517.3, 516.5  # 초점 거리
cx, cy = 318.6, 255.3  # 주점 (Principal point)
depth_scale_factor = 5000.0  # 깊이 스케일링 팩터
voxel_size = 0.01  # 샘플링 간격 (0.01m)

# 디렉토리 내 파일 목록 불러오기
depth_images = sorted(os.listdir(image_directory))
rgb_images = sorted(os.listdir(rgb_directory))

# 시각화 창 생성
vis = o3d.visualization.Visualizer()
vis.create_window()

# ViewControl 객체 가져오기
view_ctl = vis.get_view_control()

# PointCloud 객체 생성
pcd = o3d.geometry.PointCloud()

for depth_filename, rgb_filename in zip(depth_images, rgb_images):
    if depth_filename.endswith('.png') and rgb_filename.endswith('.png'):  # 확장자가 .png인 파일만 처리
        depth_image_path = os.path.join(image_directory, depth_filename)
        rgb_image_path = os.path.join(rgb_directory, rgb_filename)
        
        depth_image = cv2.imread(depth_image_path, cv2.IMREAD_UNCHANGED)
        rgb_image = cv2.imread(rgb_image_path)

        if depth_image is not None and rgb_image is not None:
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

                        # RGB 이미지에서 해당 픽셀의 색상을 가져와서 색상 설정
                        color = rgb_image[v, u] / 255.0  # OpenCV는 BGR 순서이므로 그대로 사용
                        colors.append(color)

            # Open3D 포인트 클라우드로 변환
            point_cloud = o3d.geometry.PointCloud()
            point_cloud.points = o3d.utility.Vector3dVector(points)
            point_cloud.colors = o3d.utility.Vector3dVector(colors)

            # 평면 탐지 및 제거
            plane_model, inliers = point_cloud.segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=100)
            [a, b, c, d] = plane_model
            print(f"파일: {depth_filename}, 평면 방정식: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

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
            print(f"파일: {depth_filename}, 총 군집 수: {max_label + 1}")

            colors = plt.get_cmap("tab20")(cluster_labels / (max_label if max_label > 0 else 1))
            obstacle_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])  # 클러스터에 색상 할당

            # 시각화 업데이트
            vis.clear_geometries()
            vis.add_geometry(inlier_cloud)
            vis.add_geometry(obstacle_cloud)

            # y축으로 회전
            view_ctl.rotate(0.0, 1000.0) 

            vis.poll_events()
            vis.update_renderer()

            # OpenCV로 원본 깊이 이미지 및 RGB 이미지 표시
            cv2.imshow("Depth Image", depth_image)
            cv2.imshow("RGB Image", rgb_image)

            # 프레임 간 지연 (영상처럼 송출되도록)
            if cv2.waitKey(100) & 0xFF == ord('q'):
                break
        else:
            print(f"{depth_filename} 또는 {rgb_filename} 파일을 로드할 수 없습니다.")

# 시각화 창 닫기
vis.destroy_window()
cv2.destroyAllWindows()
