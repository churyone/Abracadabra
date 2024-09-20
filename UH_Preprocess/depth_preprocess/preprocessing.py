import open3d as o3d
import numpy as np
import cv2
import matplotlib.pyplot as plt
import os
import time

def preprocess(depth_image_path, fx, fy, cx, cy, depth_scale_factor, voxel_size):
    """
    깊이 이미지를 읽어와서 포인트 클라우드를 생성하는 함수.
    - 깊이 이미지에서 각 픽셀의 깊이 값을 이용하여 3D 포인트를 계산합니다.
    - 포인트 클라우드에 색상 정보를 추가하여 시각화 시 색 구분이 가능하게 합니다.
    """
    depth_image = cv2.imread(depth_image_path, cv2.IMREAD_UNCHANGED)  # 깊이 이미지를 로드합니다.
    if depth_image is None:
        raise ValueError("Failed to load depth image.")

    height, width = depth_image.shape
    points = []
    colors = []

    min_depth = np.min(depth_image[depth_image > 0]) / depth_scale_factor
    max_depth = np.max(depth_image) / depth_scale_factor

    # 각 픽셀을 3D 포인트로 변환합니다.
    for v in range(0, height, int(voxel_size * fy)):
        for u in range(0, width, int(voxel_size * fx)):
            Z = depth_image[v, u] / depth_scale_factor
            if Z > 0:  
                X = (u - cx) * -Z / fx
                Y = (v - cy) * -Z / fy
                points.append([X, Y, Z])

                # 깊이 값에 따라 색상을 정합니다.
                normalized_depth = (Z - min_depth) / (max_depth - min_depth)
                color = [1.0 - normalized_depth, 0, normalized_depth]
                colors.append(color)

    # 포인트 클라우드를 생성하고 반환합니다.
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    point_cloud.colors = o3d.utility.Vector3dVector(colors)

    return point_cloud

def classify_plane_obstacle(point_cloud, distance_threshold=0.02, ransac_n=3, num_iterations=100, height_threshold=0.03):
    """
    RANSAC 알고리즘을 이용하여 포인트 클라우드에서 평면을 탐지하고, 
    평면과 떨어진 포인트들을 장애물로 분류하는 함수.
    - 평면에 속하는 포인트들을 inlier로, 나머지를 outlier로 분류합니다.
    - outlier 중에서 평면으로부터 일정 높이 이상 떨어진 포인트들을 장애물로 간주합니다.
    """
    # RANSAC을 사용하여 평면을 탐지합니다.
    plane_model, inliers = point_cloud.segment_plane(distance_threshold=distance_threshold, ransac_n=ransac_n, num_iterations=num_iterations)
    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

    inlier_cloud = point_cloud.select_by_index(inliers)  # 평면에 속하는 포인트들을 선택합니다.
    inlier_cloud.paint_uniform_color([0.6, 0.6, 0.6])  # 평면을 회색으로 칠합니다.

    outlier_cloud = point_cloud.select_by_index(inliers, invert=True)  # 나머지 포인트들을 선택합니다.

    # 평면으로부터의 거리 계산 후, 장애물로 분류합니다.
    distance_from_plane = np.abs(np.asarray(outlier_cloud.points) @ np.array([a, b, c]) + d) / np.linalg.norm([a, b, c])
    obstacle_indices = np.where(distance_from_plane > height_threshold)[0]
    obstacle_cloud = outlier_cloud.select_by_index(obstacle_indices)

    return inlier_cloud, obstacle_cloud, plane_model

def clustering(obstacle_cloud, eps=0.05, min_points=10):
    """
    DBSCAN 클러스터링을 사용하여 장애물 포인트 클라우드를 클러스터링하는 함수.
    - 클러스터링을 통해 서로 다른 장애물들을 구분하고, 각 클러스터에 색상을 부여합니다.
    """
    # DBSCAN 알고리즘을 이용하여 클러스터링을 수행합니다.
    cluster_labels = np.array(obstacle_cloud.cluster_dbscan(eps=eps, min_points=min_points))
    max_label = cluster_labels.max()
    print(f"Number of clusters: {max_label + 1}")

    # 각 클러스터에 색상을 부여합니다.
    colors = plt.get_cmap("tab20")(cluster_labels / (max_label if max_label > 0 else 1))
    obstacle_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])

    return obstacle_cloud

def visualize(point_cloud):
    # 카메라의 위치와 방향을 표시하는 화살표와 좌표계를 생성합니다.
    camera_center = np.array([0.0, 0.0, 0.0])
    camera_color = [1.0, 0.0, 1.0]
    camera_arrow = o3d.geometry.TriangleMesh.create_arrow(cylinder_radius=0.01, cone_radius=0.02, cylinder_height=0.1, cone_height=0.04)
    camera_arrow.paint_uniform_color(camera_color)
    camera_arrow.translate(camera_center)

    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])

    # 시각화 창을 생성하고, 평면 및 장애물 클라우드를 추가합니다.
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(point_cloud)
    vis.add_geometry(camera_arrow)
    vis.add_geometry(coordinate_frame)

    # 카메라 뷰를 설정하고 시각화를 실행합니다.
    ctr = vis.get_view_control()
    ctr.set_front([0.0, 0.0, -1.0])
    ctr.set_lookat([0.0, 0.0, 0.0])
    ctr.set_up([0.0, 1.0, 0.0])
    ctr.set_zoom(0.5)

    vis.run()
    vis.destroy_window()


def visualization(inlier_cloud, obstacle_cloud):
    """
    포인트 클라우드를 시각화하는 함수.
    - 평면과 장애물 포인트 클라우드를 함께 시각화하여, 장애물의 위치와 평면과의 관계를 확인할 수 있습니다.
    """
    # 카메라의 위치와 방향을 표시하는 화살표와 좌표계를 생성합니다.
    camera_center = np.array([0.0, 0.0, 0.0])
    camera_color = [1.0, 0.0, 1.0]
    camera_arrow = o3d.geometry.TriangleMesh.create_arrow(cylinder_radius=0.01, cone_radius=0.02, cylinder_height=0.1, cone_height=0.04)
    camera_arrow.paint_uniform_color(camera_color)
    camera_arrow.translate(camera_center)

    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])

    # 시각화 창을 생성하고, 평면 및 장애물 클라우드를 추가합니다.
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(inlier_cloud)
    vis.add_geometry(obstacle_cloud)
    vis.add_geometry(camera_arrow)
    vis.add_geometry(coordinate_frame)

    # 카메라 뷰를 설정하고 시각화를 실행합니다.
    ctr = vis.get_view_control()
    ctr.set_front([0.0, 0.0, -1.0])
    ctr.set_lookat([0.0, 0.0, 0.0])
    ctr.set_up([0.0, 1.0, 0.0])
    ctr.set_zoom(0.5)

    vis.run()
    vis.destroy_window()

def visualization_video(image_directory, fx, fy, cx, cy, depth_scale_factor, voxel_size):
    """
    주어진 디렉토리 내의 여러 깊이 이미지를 연속적으로 시각화하여, 
    마치 동영상처럼 3D 포인트 클라우드를 볼 수 있게 하는 함수.
    - 각 이미지에 대해 포인트 클라우드를 생성하고, 평면 탐지 및 장애물 분류를 수행합니다.
    - 각 프레임을 시각화 창에 표시하며, 지연 시간을 통해 동영상처럼 보이게 합니다.
    """
    depth_images = sorted(os.listdir(image_directory))

    vis = o3d.visualization.Visualizer()
    vis.create_window()

    for filename in depth_images:
        if filename.endswith('.png'):
            depth_image_path = os.path.join(image_directory, filename)
            depth_image = cv2.imread(depth_image_path, cv2.IMREAD_UNCHANGED)

            if depth_image is not None:
                # 포인트 클라우드를 생성하고 평면 탐지 및 장애물 분류를 수행합니다.
                point_cloud = preprocess(depth_image_path, fx, fy, cx, cy, depth_scale_factor, voxel_size)
                inlier_cloud, obstacle_cloud, plane_model = classify_plane_obstacle(point_cloud)

                # 시각화 창을 업데이트합니다.
                vis.clear_geometries()
                vis.add_geometry(inlier_cloud)
                vis.add_geometry(obstacle_cloud)

                # 카메라 화살표와 좌표계를 추가합니다.
                camera_center = np.array([0.0, 0.0, 0.0])
                camera_color = [1.0, 0.0, 1.0]
                camera_arrow = o3d.geometry.TriangleMesh.create_arrow(cylinder_radius=0.01, cone_radius=0.02, cylinder_height=0.1, cone_height=0.04)
                camera_arrow.paint_uniform_color(camera_color)
                camera_arrow.translate(camera_center)
                coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])

                vis.add_geometry(camera_arrow)
                vis.add_geometry(coordinate_frame)

                # 카메라 뷰 설정 및 렌더링을 갱신합니다.
                ctr = vis.get_view_control()
                ctr.set_front([0.0, 0.0, -1.0])
                ctr.set_lookat([0.0, 0.0, 0.0])
                ctr.set_up([0.0, 1.0, 0.0])
                ctr.set_zoom(0.5)

                vis.poll_events()
                vis.update_renderer()

                time.sleep(0.1)  # 프레임 간 지연을 추가하여 동영상처럼 보이게 합니다.
            else:
                print(f"{filename} 파일을 로드할 수 없습니다.")

    vis.destroy_window()
