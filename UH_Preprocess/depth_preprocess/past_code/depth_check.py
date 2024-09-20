import cv2

# Depth 이미지 파일 경로 설정
depth_img_path = 'C:/Users/User/Desktop/cap_dataset/rgbd_dataset_freiburg1_floor/rgbd_dataset_freiburg1_floor/depth/1305033527.667207.png'

# Depth 이미지 읽기
depth_img = cv2.imread(depth_img_path, cv2.IMREAD_UNCHANGED)  # 16비트로 읽기

if depth_img is not None:
    # 이미지의 중앙 픽셀 값 출력
    height, width = depth_img.shape
    center_pixel_value = depth_img[height//2, width//2]
    print(f"Depth 이미지의 중앙 픽셀 값: {center_pixel_value}")

    # 이미지의 모든 픽셀 값의 최대값과 최소값 출력
    print(f"최소 Depth 값: {depth_img.min()}, 최대 Depth 값: {depth_img.max()}")
else:
    print("Depth 이미지를 불러올 수 없습니다.")
