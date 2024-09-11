//
//  ContentView.swift
//  Abracadabra
//
//  Created by 박진우 on 8/12/24.
//

import SwiftUI

struct ContentView: View {
    @StateObject private var locationManager = LocationManager()
    @StateObject private var motionDataView = MotionDataView()  // MotionDataView가 ObservableObject를 준수
    @State private var depthData: CVPixelBuffer? = nil
    @State private var webSocketManager = WebSocketManager()  // WebSocketManager 추가
    @State private var timer: Timer? = nil
    var body: some View {
        VStack {
            if let location = locationManager.location {
                Text("Latitude: \(location.coordinate.latitude), Longitude: \(location.coordinate.longitude)")
            } else {
                Text("Getting location...")
            }

            MotionDataDisplayView(motionDataView: motionDataView)
                .frame(height: 200)
                .padding(.bottom, 20)

            ARDepthCameraView(depthData: $depthData)
                .frame(width: UIScreen.main.bounds.width, height: 300)

            if let depthData = depthData {
                DepthDataView(depthData: depthData)
                    .frame(height: 300)
            } else {
                Text("Depth data not available")
                    .frame(height: 300)
            }
        }
        .onAppear {
            startSendingData()  // 화면이 나타날 때 타이머 시작
        }
        .onDisappear {
            stopSendingData()
            motionDataView.stopMotionUpdates()  // MotionDataView에서 업데이트 중지
        }
        .padding()
    }
    private func startSendingData() {
        // 타이머를 5초 간격으로 설정 (5초마다 데이터 전송)
        timer = Timer.scheduledTimer(withTimeInterval: 2, repeats: true) { _ in
            if let depthData = depthData {
                // 이미지도 전송
                if let uiImage = DepthDataView(depthData: depthData).convertToColorMappedUIImage(pixelBuffer: depthData) {
                    sendData(depthData, imuData: collectIMUData(), image: uiImage)  // 데이터 전송
                } else {
                    print("DepthData 이미지 변환 실패")
                }
            }
        }
    }
    
    private func stopSendingData() {
            timer?.invalidate()
            timer = nil
        }
    
    private func collectIMUData() -> [String: Any] {
        var imuData: [String: Any] = [:]

        if let accelData = motionDataView.accelerometerData {
            imuData["accelerometer"] = ["x": accelData.acceleration.x, "y": accelData.acceleration.y, "z": accelData.acceleration.z]
        }

        if let gyroData = motionDataView.gyroData {
            imuData["gyroscope"] = ["x": gyroData.rotationRate.x, "y": gyroData.rotationRate.y, "z": gyroData.rotationRate.z]
        }

        if let magnetometerData = motionDataView.magnetometerData {
            imuData["magnetometer"] = ["x": magnetometerData.magneticField.x, "y": magnetometerData.magneticField.y, "z": magnetometerData.magneticField.z]
        }
        return imuData
    }

    private func sendData(_ depthData: CVPixelBuffer, imuData: [String: Any], image: UIImage) {
            // LiDAR 데이터를 Data로 변환
            CVPixelBufferLockBaseAddress(depthData, .readOnly)
            defer { CVPixelBufferUnlockBaseAddress(depthData, .readOnly) }

            let width = CVPixelBufferGetWidth(depthData)
            let height = CVPixelBufferGetHeight(depthData)
            let depthDataSize = width * height * MemoryLayout<Float32>.size
            let depthDataPointer = CVPixelBufferGetBaseAddress(depthData)
            let data = Data(bytes: depthDataPointer!, count: depthDataSize)
            guard let imageData = image.jpegData(compressionQuality: 0.8) else {
                print("이미지 데이터를 얻을 수 없습니다.")
                return
            }
            let imageBase64String = imageData.base64EncodedString()

            // LiDAR 및 IMU 데이터를 JSON으로 묶어 전송
            let payload: [String: Any] = [
                "depthData": data.base64EncodedString(),  // LiDAR 데이터 Base64 인코딩
                "imuData": imuData,
                "gpsData": [
                    "latitude": motionDataView.latitude ?? 0.0,
                    "longitude": motionDataView.longitude ?? 0.0
                ],
                "width" : width,
                "height" : height,
                "imageData": imageBase64String
            ]
        
            // JSON 데이터를 WebSocket으로 전송
            if let jsonData = try? JSONSerialization.data(withJSONObject: payload, options: []),
               let jsonString = String(data: jsonData, encoding: .utf8) {
                webSocketManager.sendMessage(jsonString)
            }
        }
}
