import cv2
import os
import numpy as np

# Depth 이미지들이 있는 디렉토리 경로 설정
depth_dir = 'C:/Users/User/Desktop/cap_dataset/rgbd_dataset_freiburg1_floor/rgbd_dataset_freiburg1_floor/depth'

# 디렉토리 내 파일 목록 불러오기
depth_images = sorted(os.listdir(depth_dir))

# 장애물로 간주할 거리 임계값 (Scaling: 5000)
threshold_distance = 6000  # 1.2미터

# 이미지 파일들을 순차적으로 읽어와서 처리
for depth_img_name in depth_images:
    # 각 이미지의 전체 경로 생성
    depth_img_path = os.path.join(depth_dir, depth_img_name)
    
    # Depth 이미지 읽기
    depth_img = cv2.imread(depth_img_path, cv2.IMREAD_UNCHANGED)
    
    if depth_img is None:
        print(f"이미지를 읽을 수 없습니다: {depth_img_path}")
        continue
    
    # 임계값 이하인 픽셀을 장애물로 간주하여 이진 마스크 생성
    obstacle_mask = np.where(depth_img < threshold_distance, 255, 0).astype(np.uint8)
    
    # 결과 시각화
    cv2.imshow('Depth Image', depth_img)  # 원본 Depth 이미지 보기
    cv2.imshow('Obstacle Mask', obstacle_mask)  # 장애물 마스크 이미지 보기
    
    # 키 입력 대기 (예: ESC 키를 누르면 종료, 다른 키를 누르면 다음 이미지로 넘어감)
    key = cv2.waitKey(500)  # 500ms 동안 대기 (이 시간을 조절하여 속도를 변경할 수 있음)
    if key == 27:  # ESC 키의 ASCII 코드가 27입니다.
        break

# 모든 창 닫기
cv2.destroyAllWindows()
