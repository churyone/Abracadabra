from func_utils import API, FrameProcessor
from ultralytics import YOLO
import psycopg2
import requests
import xml.etree.ElementTree as ET
import threading
from queue import Queue
import easyocr
import os
import sys
import numpy as np
import time
import json
import cv2

start_time = time.time()


# # 환경 변수 설정
# os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = '/home/minseokim521/catkin_ws/src/bus/zippy-brand-429513-k7-6ef67897540d.json'

# 모델 경로 설정


yolo_image_path = "/home/LOE/workspace/server/image_to_send.png" 
txt_path = "/home/LOE/workspace/server/message.txt"
image_path = "/home/LOE/workspace/server/rgb_frame.png"
model_path = "/home/LOE/workspace/server/Blind_Bus_Support-bbs-/models/best_3000_s.pt"
file_path = '/home/LOE/workspace/server/sensor_data.json'

with open(file_path, 'r') as file:
    data = json.load(file)

'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
------------------------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------------------------
                                                    YOLO + OCR section
------------------------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------------------------
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
yolo_ocr_start = time.time()



model = YOLO(model_path)
easy_ocr = easyocr.Reader(['en'], gpu=True)

model.overrides['verbose'] = False

frame = cv2.imread(image_path)
print(frame.shape)
height, width = frame.shape[:2]
print(f"이미지가 성공적으로 로드되었습니다. 해상도: {width}x{height}")

# 번호판 인식 클래스 이름 설정 및 인덱스 확인
plate_class_names = ['front_num', 'side_num', 'back_num']
plate_class_indices = [idx for idx, name in model.names.items() if name in plate_class_names]

# YOLO 모델로 프레임 처리
padding = 5
min_confidence = 0.8
frame_processor = FrameProcessor(
    model, plate_class_indices, easy_ocr,
    width, height, padding, min_confidence
)
ocr_number = []




ocr_number.append(frame_processor.process_frame(frame, yolo_image_path))
if ocr_number[0]:
    print(f"OCR 결과로 추출된 번호판: {ocr_number}")
else:
    msg1 = "아무 번호도 인식되지 않았습니다."
    print("아무 번호도 인식되지 않았습니다.")
    with open(file_path, "w", encoding="utf-8") as file:
        file.write(msg1)
    exit()

# # ocr결과에서 None을 제거
# filtered_ocr_numbers = [num for num in ocr_number if num is not None]


# print(f"filtered_ocr_number : {filtered_ocr_numbers}")


    
        



#걸린 시간 출력
yolo_ocr_end = time.time()

print(f"Yolo + OCR section took {yolo_ocr_end - yolo_ocr_start} seconds. ")

'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
------------------------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------------------------
                                                        API section
------------------------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------------------------
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
api_start = time.time()


# GPS 데이터를 받아오고 그 값을 변수에 저장

# latitude, longitude = data['gpsData']['latitude'], data['gpsData']['longitude']


latitude, longitude = 37.5158657465, 126.90509208
bus_api = API()

Bus_num = ocr_number[0]

#데이터 베이스 상에서 버스 번호와 정류소 이름에 해당하는 id값 가져오기
bus_result = bus_api.database_query('bus', 'routeid', 'bus_id', Bus_num)
station_list = bus_api.database_query_specific_column("station", 'node_id')
station_name_list = bus_api.database_query_specific_column("station", 'station_name')
X_locations = bus_api.database_query_specific_column("station", 'X_location')
Y_locations = bus_api.database_query_specific_column("station", 'Y_location')

# 리스트 평탄화
X_locations = [x[0] for x in X_locations]
Y_locations = [y[0] for y in Y_locations]

# 가장 가까운 정류소 인덱스 찾기
index = bus_api.find_nearest_index(longitude, latitude, X_locations, Y_locations)
station_name = station_name_list[index]
station_id = station_list[index]

print(f"가장 가까운 버스 정류장의 이름 :{station_name}")

if bus_result == None:
    print("OCR상의 버스 번호가 DB와 일치하지 않습니다.")
    exit()
# api 상에서 station id에 해당하는 정류소에 운행하는 버스정보 가져오기
response2 = bus_api.station_bus_list(station_id[0])

#xml 값을 가져옴
root2 = ET.fromstring(response2)

# 정류소에서 운행하는 버스 이름 정보 리스트
bus_list = bus_api.find_xml_val(root2, "busRouteAbrv")

# 버스 리스트에서 인식한 버스 정보가 있는지 찾음
index, result = bus_api.find_api_val(bus_list, Bus_num)

isArrive1 = []
arrmsg1_list = []
arrmsg2_list = []

if result:
    isArrive1 = bus_api.find_xml_val(root2, "isArrive1")
    arrmsg1_list = bus_api.find_xml_val(root2, "arrmsg1")
    arrmsg2_list = bus_api.find_xml_val(root2, "arrmsg2")
else:
    exit()
msg1, msg2 = 0,0

# 변수 출력
msg1 = f"가장 가까운 정류장은 {station_name}입니다."

msg2 = f"눈앞의 버스는 {Bus_num}번 입니다"
    
msg3 = "첫번째 버스 도착 예정시간 :" + arrmsg1_list[index] 
print(msg1)
print(msg2)
print(msg3)
with open(txt_path, "w", encoding="utf-8") as file:
    file.write(msg1 + "\n" + msg2 + "\n" + msg3)

api_end = time.time()
print(f"API section took {api_end - api_start} seconds.")

