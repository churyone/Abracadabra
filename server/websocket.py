import numpy as np
import cv2
import struct
import base64
from fastapi import FastAPI, WebSocket
from fastapi.responses import HTMLResponse, FileResponse
import json
import os
import time

app = FastAPI()

# 저장할 경로 설정
SENSOR_DATA_FILE = "sensor_data.json"
IMAGE_FILE = "depth_map.png"  # 이미지를 저장할 경로

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    print("클라이언트 연결됨")

    try:
        while True:
            # 클라이언트로부터 JSON 데이터 수신
            data = await websocket.receive_text()

            # JSON 데이터를 파싱하여 각 센서 데이터를 출력
            json_data = json.loads(data)
            depth_data_base64 = json_data.get("depthData")
            imu_data = json_data.get("imuData")
            width = json_data.get("width")
            height = json_data.get("height")
            gps_data = json_data.get("gpsData")
            img_data_base64 = json_data.get("imageData")
            time_data = json_data.get("timestamp")

            depth_data_floats = None  # 변환된 32비트 부동소수점 값 리스트 저장용

            # depthData가 base64로 인코딩된 경우 디코딩 및 변환
            if depth_data_base64:
                try:
                    depth_data = base64.b64decode(depth_data_base64)
                    print(f"디코딩된 LiDAR 데이터 크기: {len(depth_data)} 바이트")
                    # 32비트 부동소수점(float32)로 변환
                    depth_data_floats = struct.unpack('f' * (len(depth_data) // 4), depth_data)

                    
                    print(f"첫 10개의 뎁스 값: {depth_data_floats[:10]}")  # 첫 10개의 값을 출력
                    print(f"리스트의 길이 {len(depth_data_floats)}")

                    # 뎁스 데이터를 이미지로 변환하여 저장
                    depth_map_to_image(img_data_base64, IMAGE_FILE)

                except Exception as e:
                    print(f"LiDAR 데이터 디코딩 중 오류 발생: {e}")
                    depth_data_floats = None

            print(f"수신한 IMU 데이터: {imu_data}")
            print(f"수신한 GPS 데이터: {gps_data}")
            print(f"데이터의 가로 세로 길이 {width}x{height}")

            # 수신한 데이터를 파일에 저장
            save_sensor_data({
                "depthData": depth_data_floats,  # 32비트 부동소수점 값으로 변환된 리스트를 저장
                "imuData": imu_data,
                "gpsData": gps_data,
                "width" : width,
                "height" : height,
                "time_data" : time_data
            })

            # 클라이언트에게 수신한 데이터를 다시 전송 (HTML에서 표시 가능)
            await websocket.send_text(data)

    except Exception as e:
        print(f"연결 종료 오류: {e}")
    finally:
        print("클라이언트 연결 종료")
        await websocket.close()



def depth_map_to_image(img_data_base64, output_image_file="depth_map.png"):
    # Base64로 인코딩된 이미지 데이터를 디코딩
    img_data = base64.b64decode(img_data_base64)

    # 1차원 배열로 변환
    img_array = np.frombuffer(img_data, dtype=np.uint8)

    # OpenCV를 사용하여 1차원 배열을 이미지로 디코딩
    depth_image = cv2.imdecode(img_array, cv2.IMREAD_UNCHANGED)

    if depth_image is None:
        print("이미지 디코딩 실패")
        return



    # # 깊이 데이터를 0에서 1 범위로 정규화 (min/max 범위를 0~255로 정규화)
    # normalized_depth = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)

    # # 0에서 255 범위의 8비트 이미지로 변환
    # depth_image_8bit = np.uint8(normalized_depth)

    # 컬러 맵 적용 (빨간색에서 파란색으로)


    # 이미지를 파일로 저장
    cv2.imwrite(output_image_file, depth_image)
    print(f"컬러 맵핑된 깊이 이미지가 {output_image_file}로 저장되었습니다.")



# HTML 페이지로 클라이언트 제공
@app.get("/")
async def get():
    return HTMLResponse('''
    <!DOCTYPE html>
    <html lang="en">
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>WebSocket 클라이언트</title>
    </head>
    <body>
        <h1>실시간 센서 데이터 및 Depth Map</h1>

        <!-- LiDAR 데이터 표시 부분 제거 -->
        <!-- <p><strong>LiDAR 데이터:</strong> <span id="depthData">대기 중...</span></p> -->

        <p><strong>IMU 데이터:</strong> <span id="imuData">대기 중...</span></p>
        <p><strong>GPS 데이터:</strong> <span id="gpsData">대기 중...</span></p>

        <h2>Depth Map</h2>
        <img id="depthMapImage" src="/depth_map_image" alt="Depth Map" width="590" height="800">

        <script>
            let ws = new WebSocket("ws://localhost:8000/ws");

            // WebSocket을 통해 서버로부터 데이터를 받으면 HTML 업데이트
            ws.onmessage = function(event) {
                const data = JSON.parse(event.data);
                console.log("수신한 데이터:", data);

                // LiDAR 데이터를 표시하지 않음
                // document.getElementById('depthData').textContent = JSON.stringify(data.depthData) || 'N/A';

                document.getElementById('imuData').textContent = JSON.stringify(data.imuData) || 'N/A';

                if (data.gpsData) {
                    document.getElementById('gpsData').textContent = `Latitude: ${data.gpsData.latitude}, Longitude: ${data.gpsData.longitude}` || 'N/A';
                }
            };

            ws.onopen = function(event) {
                console.log("WebSocket 연결 성공");
            };

            ws.onclose = function(event) {
                console.log("WebSocket 연결 종료");
            };

            ws.onerror = function(event) {
                console.error("WebSocket 오류 발생:", event);
            };

            // 1초마다 Depth Map 이미지와 센서 데이터 업데이트
            function updateImageAndData() {
                // 이미지 갱신 (1초마다 새로운 이미지 요청)
                document.getElementById('depthMapImage').src = `/depth_map_image?timestamp=${new Date().getTime()}`;

                // JSON 파일에서 센서 데이터 요청
                fetch("/sensor_data.json")
                    .then(response => response.json())
                    .then(sensorData => {
                        // LiDAR 데이터를 표시하지 않음
                        // document.getElementById('depthData').textContent = sensorData.depthData || 'N/A';

                        document.getElementById('imuData').textContent = JSON.stringify(sensorData.imuData) || 'N/A';

                        if (sensorData.gpsData) {
                            document.getElementById('gpsData').textContent = `Latitude: ${sensorData.gpsData.latitude}, Longitude: ${sensorData.gpsData.longitude}`;
                        }
                    })
                    .catch(error => {
                        console.error("센서 데이터 갱신 중 오류 발생:", error);
                    });
            }

            // 1초마다 이미지 및 데이터 업데이트
            setInterval(updateImageAndData, 500);  // 1000ms = 1초
        </script>
    </body>
    </html>
    ''')

# 이미지를 클라이언트에게 제공하는 엔드포인트
@app.get("/depth_map_image")
async def get_depth_map_image():
    return FileResponse(IMAGE_FILE)


SENSOR_DATA_FILE = "sensor_data.json"  # 실제 JSON 파일 경로

@app.get("/sensor_data.json")
async def get_sensor_data():
    return FileResponse(SENSOR_DATA_FILE)

# # 수신한 데이터를 'sensor_data.json' 파일에 저장하는 함수
# def save_sensor_data(sensor_data):
#     try:
#         with open(SENSOR_DATA_FILE, 'w') as file:
#             json.dump(sensor_data, file, indent=4)  # 32비트 부동소수점 값 리스트를 JSON 파일로 저장
#         print(f"센서 데이터를 {SENSOR_DATA_FILE}에 저장했습니다.")
#     except Exception as e:
#         print(f"데이터 저장 중 오류 발생: {e}")

# JSON 파일을 고유한 이름으로 저장하는 함수
def save_sensor_data(sensor_data):
    # 타임스탬프를 사용하여 파일 이름에 추가
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    file_name = f"sensor_data_{timestamp}.json"
    try:
        with open(file_name, 'w') as file:
            json.dump(sensor_data, file, indent=4)  # 32비트 부동소수점 값 리스트를 JSON 파일로 저장
        print(f"센서 데이터를 {file_name}에 저장했습니다.")
    except Exception as e:
        print(f"데이터 저장 중 오류 발생: {e}")