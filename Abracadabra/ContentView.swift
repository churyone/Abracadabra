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
                    //.rotationEffect(.degrees(90))  // 시계방향으로 90도 회전

                // Depth Data 및 IMU 데이터를 WebSocket을 통해 서버로 전송하는 버튼
               /**
                Button(action: {
                    sendData(depthData, imuData: collectIMUData())
                }) {
                    Text("Send Depth & IMU Data")
                        .foregroundColor(.white)
                        .padding()
                        .background(Color.blue)
                        .cornerRadius(8)
                }
                .padding(.top, 20)
                */
            } else {
                Text("Depth data not available")
                    .frame(height: 300)
            }
        }
        .onDisappear {
            motionDataView.stopMotionUpdates()  // MotionDataView에서 업데이트 중지
        }
        .padding()
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

    private func sendData(_ depthData: CVPixelBuffer, imuData: [String: Any]) {
        // Convert CVPixelBuffer to Data
        CVPixelBufferLockBaseAddress(depthData, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthData, .readOnly) }

        let width = CVPixelBufferGetWidth(depthData)
        let height = CVPixelBufferGetHeight(depthData)
        let depthDataSize = width * height * MemoryLayout<Float32>.size
        let depthDataPointer = CVPixelBufferGetBaseAddress(depthData)

        let data = Data(bytes: depthDataPointer!, count: depthDataSize)

        // Create a dictionary with depth data and IMU data
        let payload: [String: Any] = [
            "depthData": data.base64EncodedString(),  // Encode depth data as Base64
            "imuData": imuData
        ]

        // Convert the dictionary to JSON string
        if let jsonData = try? JSONSerialization.data(withJSONObject: payload, options: []),
           let jsonString = String(data: jsonData, encoding: .utf8) {
            // Send the JSON string via WebSocket
            webSocketManager.sendMessage(jsonString)
        }
    }
}
