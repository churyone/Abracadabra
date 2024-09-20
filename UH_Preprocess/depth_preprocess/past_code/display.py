import cv2
import os

# RGB 이미지와 Depth 이미지가 있는 디렉토리 경로 설정
rgb_dir = 'C:/Users/User/Desktop/cap_dataset/rgbd_dataset_freiburg1_floor/rgbd_dataset_freiburg1_floor/rgb'
depth_dir = 'C:/Users/User/Desktop/cap_dataset/rgbd_dataset_freiburg1_floor/rgbd_dataset_freiburg1_floor/depth'


# 이미지 파일 목록 불러오기
rgb_images = sorted(os.listdir(rgb_dir))
depth_images = sorted(os.listdir(depth_dir))

# 이미지 표시
for rgb_img_name, depth_img_name in zip(rgb_images, depth_images):
    # RGB 이미지 읽기
    rgb_img_path = os.path.join(rgb_dir, rgb_img_name)
    rgb_img = cv2.imread(rgb_img_path)
    
    # Depth 이미지 읽기
    depth_img_path = os.path.join(depth_dir, depth_img_name)
    depth_img = cv2.imread(depth_img_path, cv2.IMREAD_UNCHANGED)  # Depth 이미지는 일반적으로 16비트 이미지

    # 이미지 표시
    cv2.imshow('RGB Image', rgb_img)
    cv2.imshow('Depth Image', depth_img)
    
    # 키 입력 대기 (예: 'q' 키를 누르면 종료)
    if cv2.waitKey(100) & 0xFF == ord('q'):
        break

# 모든 창 닫기
cv2.destroyAllWindows()
