import cv2
import numpy as np
import os

# Depth 이미지들이 있는 디렉토리 경로 설정
depth_dir = 'C:/Users/User/Desktop/cap_dataset/rgbd_dataset_freiburg1_floor/rgbd_dataset_freiburg1_floor/depth'

# 디렉토리 내 파일 목록 불러오기
depth_images = sorted(os.listdir(depth_dir))

def find_direction_based_on_depth(depth_image, roi_top=50, roi_bottom=430, roi_left=50, roi_right=570):
    height, width = depth_image.shape
    
    # 관심 영역(ROI) 설정 (위쪽, 아래쪽, 왼쪽, 오른쪽 일부 제외)
    roi = depth_image[roi_top:roi_bottom, roi_left:roi_right]
    
    # 관심 영역을 세로로 3등분
    roi_width = roi.shape[1]
    left_part = roi[:, :roi_width//3]
    center_part = roi[:, roi_width//3:2*roi_width//3]
    right_part = roi[:, 2*roi_width//3:]
    
    # 각 파트의 평균 뎁스 값 계산
    left_mean_depth = np.mean(left_part)
    center_mean_depth = np.mean(center_part)
    right_mean_depth = np.mean(right_part)
    
    # 가장 큰 평균 뎁스 값을 가진 방향을 찾음
    directions = {'Left': left_mean_depth, 'Straight': center_mean_depth, 'Right': right_mean_depth}
    best_direction = max(directions, key=directions.get)
    
    return best_direction, directions, roi

# 각 이미지에 대해 방향을 결정
for depth_image_file in depth_images:
    depth_image_path = os.path.join(depth_dir, depth_image_file)
    depth_image = cv2.imread(depth_image_path, cv2.IMREAD_UNCHANGED)
    
    if depth_image is not None:
        print(f"Processing {depth_image_file}...")
        best_direction, mean_depths, roi = find_direction_based_on_depth(depth_image)
        print(f"Best direction: {best_direction}")
        print(f"Mean depths - Left: {mean_depths['Left']:.2f}, Straight: {mean_depths['Straight']:.2f}, Right: {mean_depths['Right']:.2f}")
        
        # 관심 영역(ROI) 및 각 파트를 시각화
        cv2.imshow('ROI', roi)
        cv2.imshow('Left Part', roi[:, :roi.shape[1]//3])
        cv2.imshow('Center Part', roi[:, roi.shape[1]//3:2*roi.shape[1]//3])
        cv2.imshow('Right Part', roi[:, 2*roi.shape[1]//3:])
        
        # 100ms 대기 후 다음 이미지로 자동 넘어감
        if cv2.waitKey(100) & 0xFF == ord('q'):
            break
    else:
        print(f"Failed to load {depth_image_file}")

cv2.destroyAllWindows()
