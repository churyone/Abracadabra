import paho.mqtt.client as mqtt
import time
from preprocessing import preprocess, classify_plane_obstacle, visualize, visualization
from projection import project_to_2d, cluster_and_fit_shapes_2d, visualize_shapes_2d
from path_decision import visualize_optimal_direction  # 최적 방향 시각화 함수 사용

# MQTT 클라이언트 설정
def on_connect(client, userdata, flags, rc):
    print("서버가 브로커에 연결되었습니다.")
    client.subscribe("heartbeat")  # 하트비트 토픽 구독

def on_message(client, userdata, msg):
    raspberry_ip = msg.payload.decode()
    print(f"수신된 하트비트 IP: {raspberry_ip}")

client = mqtt.Client()

client.on_connect = on_connect
client.on_message = on_message

broker_ip = '103.218.162.56'  # 서버의 공인 IP (MQTT 브로커가 실행되는 IP)

# 브로커에 연결
client.connect(broker_ip, 2934, 60)

# MQTT로 벡터 데이터를 전송하는 함수
def send_vector_data(vector_data, topic="vector"):
    message = f"Vector data: {vector_data}"
    client.publish(topic, message)
    print(f"벡터 데이터 발행: {message}")

# main 함수
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
    
    # 5. 2D 투영된 포인트 클러스터링 및 도형 생성
    clusters, shapes = cluster_and_fit_shapes_2d(projected_points)
    visualize_shapes_2d(projected_points, shapes)

    # 6. 최적 방향 계산 및 시각화
    optimal_vector = visualize_optimal_direction(shapes)
    
    # 7. MQTT로 최적 벡터 전송
    send_vector_data(optimal_vector, topic="vector/optimal_direction")

if __name__ == "__main__":
    # 카메라 파라미터 및 이미지 경로 설정
    depth_image_path = '/home/hosuripa/Abracadabra_server/server/colored_depth_map.png'
    image_directory = '/home/hosuripa/Abracadabra_server/server/'

    fx, fy = 517.3, 516.5  # 카메라의 초점 거리
    cx, cy = 318.6, 255.3  # 카메라의 주점 (Principal Point)
    depth_scale_factor = 5000.0  # 깊이 값 스케일링 팩터
    voxel_size = 0.01  # 샘플링 간격 (0.01m)

    # main 함수 실행 및 MQTT 클라이언트 시작
    main(depth_image_path, image_directory, fx, fy, cx, cy, depth_scale_factor, voxel_size)
    client.loop_forever()  # 계속해서 메시지를 기다림
