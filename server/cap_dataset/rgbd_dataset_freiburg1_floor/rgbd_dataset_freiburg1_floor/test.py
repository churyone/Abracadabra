import numpy as np
import cv2


image = cv2.imread("/Users/idongmyeong/Downloads/cap_dataset/rgbd_dataset_freiburg1_floor/rgbd_dataset_freiburg1_floor/depth/1305033530.464959.png")
image_array = np.array(image[:,:,0])
# 이미지 파일 로드
depth_image = cv2.imread("/Users/idongmyeong/Downloads/cap_dataset/rgbd_dataset_freiburg1_floor/rgbd_dataset_freiburg1_floor/depth/1305033530.464959.png", cv2.IMREAD_UNCHANGED)

# 데이터 타입 확인
print(depth_image.dtype)  # np.uint8 (8비트) 또는 np.uint16 (16비트) 출력됨


print(f"이미지 행렬값 :{image_array[:,:]}")
print("shape", image_array.shape)
