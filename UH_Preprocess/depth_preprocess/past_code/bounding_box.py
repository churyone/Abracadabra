import open3d as o3d
import numpy as np
import cv2
import matplotlib.pyplot as plt

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

    # Bounding Box 추가
    bounding_boxes = []
    for i in range(max_label + 1):
        cluster_indices = np.where(cluster_labels == i)[0]
        cluster_cloud = obstacle_cloud.select_by_index(cluster_indices)
        bbox = cluster_cloud.get_axis_aligned_bounding_box()
        bbox.color = (1, 0, 0)  # Bounding Box 색상 (빨간색)
        bounding_boxes.append(bbox)

    return inlier_cloud, obstacle_cloud, bounding_boxes

def visualize_point_cloud(inlier_cloud, obstacle_cloud, bounding_boxes):
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    # 시각화 업데이트
    vis.add_geometry(inlier_cloud)
    vis.add_geometry(obstacle_cloud)
    
    # Bounding Box 추가
    for bbox in bounding_boxes:
        vis.add_geometry(bbox)

    # ViewControl 객체 가져오기
    view_ctl = vis.get_view_control()

    # 시점 변경 - y축 회전
    view_ctl.rotate(0.0, 1000.0)

    vis.poll_events()
    vis.update_renderer()

    vis.run()  # 사용자가 닫을 때까지 창 유지
    vis.destroy_window()

def main():
    depth_image_path = 'C:/Users/User/Desktop/cap_dataset/rgbd_dataset_freiburg1_floor/rgbd_dataset_freiburg1_floor/depth/1305033532.903424.png'
    fx, fy = 517.3, 516.5  # 초점 거리
    cx, cy = 318.6, 255.3  # 주점 (Principal point)
    depth_scale_factor = 5000.0  # 깊이 스케일링 팩터
    voxel_size = 0.01  # 샘플링 간격 (0.01m)

    points, colors = load_depth_image(depth_image_path, fx, fy, cx, cy, depth_scale_factor, voxel_size)
    if points is not None and colors is not None:
        inlier_cloud, obstacle_cloud, bounding_boxes = process_point_cloud(points, colors)
        visualize_point_cloud(inlier_cloud, obstacle_cloud, bounding_boxes)

if __name__ == "__main__":
    main()
