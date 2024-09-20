import numpy as np
import matplotlib.pyplot as plt

def calculate_angle_and_distance(point):
    """
    주어진 포인트가 원점(0, 0)과 이루는 선의 각도와 거리를 계산하는 함수.
    각도는 +x축 기준으로 계산합니다.
    """
    x, y = point
    distance = np.sqrt(x**2 + y**2)  # 원점으로부터의 거리
    angle = np.arctan2(y, x)  # +x축 기준 각도 (라디안)
    
    return angle, distance

def find_optimal_direction(shapes):
    """
    shapes 리스트에서 각 도형의 최소 각도와 최대 각도를 피하고, 
    +y축 방향(90도, π/2 라디안)과 가장 가까운 방향을 선택하는 함수.
    
    Parameters:
    - shapes: 각 클러스터의 경계 도형 (점들의 좌표 배열 리스트)
    
    Returns:
    - optimal_angle: 선택된 최적의 이동 각도 (라디안)
    """
    origin = np.array([0, 0])
    y_direction = np.pi / 2  # +y축 방향 (90도, π/2 라디안)

    # 각 장애물 클러스터에서 최소 및 최대 각도를 계산
    exclusion_angles = []
    for shape in shapes:
        angles = [calculate_angle_and_distance(point)[0] for point in shape]
        min_angle = min(angles)
        max_angle = max(angles)
        exclusion_angles.append((min_angle, max_angle))

    # 피해야 할 각도 범위를 제외하고 가능한 각도 찾기
    candidate_angles = np.linspace(-np.pi, np.pi, 360)
    valid_angles = []

    for angle in candidate_angles:
        is_valid = True
        for min_angle, max_angle in exclusion_angles:
            if min_angle <= angle <= max_angle:
                is_valid = False
                break
        if is_valid:
            valid_angles.append(angle)

    # +y축 방향과 가장 가까운 각도를 선택
    optimal_angle = min(valid_angles, key=lambda a: abs(a - y_direction))
    
    return optimal_angle

def visualize_optimal_direction(shapes):
    """
    최적의 방향을 시각화하는 함수. 각 shape들의 경계와 +y축을 기준으로 최적의 이동 방향을 계산한 후,
    그 방향을 원점에서 화살표로 표시합니다.
    
    Parameters:
    - shapes: Convex Hull을 통해 얻어진 각 클러스터의 경계 도형 (점들의 좌표 배열 리스트)
    """
    plt.figure(figsize=(8, 8))

    # 원점 정의
    origin = np.array([0, 0])

    # 각 shape을 시각화
    for shape in shapes:
        plt.scatter(shape[:, 0], shape[:, 1], c='red', marker='o', s=20, label='Shape Points')
    
    # 최적의 방향 계산
    optimal_angle = find_optimal_direction(shapes)
    
    # 최적의 방향을 화살표로 시각화
    optimal_direction = np.array([np.cos(optimal_angle), np.sin(optimal_angle)])
    print(optimal_direction)
    plt.arrow(origin[0], origin[1], optimal_direction[0], optimal_direction[1], 
              head_width=0.1, head_length=0.1, fc='blue', ec='blue', label='Optimal Direction')

    # 원점과 +y축 시각화
    y_direction = np.array([0, 1])
    plt.arrow(origin[0], origin[1], y_direction[0], y_direction[1], 
              head_width=0.05, head_length=0.1, fc='green', ec='green', label='+y Direction')

    plt.title("Optimal Direction Avoiding Obstacles")
    plt.xlabel("Local X")
    plt.ylabel("Local Y")
    plt.grid(True)
    plt.legend()
    plt.show()
