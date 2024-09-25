//
//  ContentView.swift
//  Abracadabra
//
//  Created by 박진우 on 8/12/24.
//

import SwiftUI

struct MainView: View {
    @StateObject private var locationManager = LocationManager()
    @StateObject private var motionDataView = MotionDataView()
    @State private var depthData: CVPixelBuffer? = nil
    @EnvironmentObject var webSocketManager: WebSocketManager
    @State private var timer: Timer? = nil
    @State private var cameraIntrinsics: (fx: Float, fy: Float, cx: Float, cy: Float)? = nil
    @State private var rgbImage: CIImage? = nil

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

            ARDepthCameraView(depthData: $depthData, rgbImage: $rgbImage,cameraIntrinsics: $cameraIntrinsics)
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
        // 카메라 파라미터 언랩핑
            guard let cameraIntrinsics = cameraIntrinsics else {
                print("카메라 내재 파라미터가 없습니다.")
                return
            }
                
            var rgbImageBase64String: String = ""
            if let rgbImage = rgbImage {
                let context = CIContext()
                if let rgbCGImage = context.createCGImage(rgbImage, from: rgbImage.extent) {
                    let rgbUIImage = UIImage(cgImage: rgbCGImage)
                    if let rgbImageData = rgbUIImage.jpegData(compressionQuality: 0.8) {
                        rgbImageBase64String = rgbImageData.base64EncodedString()
                    } else {
                        print("RGB 이미지 데이터를 얻을 수 없습니다.")
                    }
                }
            }

        
            let imageBase64String = imageData.base64EncodedString()
        
            let formatter = ISO8601DateFormatter()
            formatter.formatOptions = [.withInternetDateTime, .withFractionalSeconds]
            let timestamp = formatter.string(from: Date())
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
                "imageData": imageBase64String,
                "rgbImageData": rgbImageBase64String,
                "timestamp": timestamp,
                "cameraIntrinsics": [
                    "fx": cameraIntrinsics.fx,
                    "fy": cameraIntrinsics.fy,
                    "cx": cameraIntrinsics.cx,
                    "cy": cameraIntrinsics.cy
                ]
            ]
            
            // JSON 데이터를 WebSocket으로 전송
            if let jsonData = try? JSONSerialization.data(withJSONObject: payload, options: []),
               let jsonString = String(data: jsonData, encoding: .utf8) {
                webSocketManager.sendMessage(jsonString)
            }
        }
}
    

struct SecondView: View {
    @EnvironmentObject var webSocketManager: WebSocketManager

    var body: some View {
        VStack {
            if let image = webSocketManager.receivedImage {
                Image(uiImage: image)
                    .resizable()
                    .aspectRatio(contentMode: .fit)
                    .frame(width: 300, height: 300)
            } else {
                Text("이미지를 기다리는 중...")
                    .padding()
            }

            if let text = webSocketManager.receivedText {
                Text(text)
                    .padding()
            } else {
                Text("텍스트를 기다리는 중...")
                    .padding()
            }
        }
        .onAppear {
            // 필요 시 추가 설정
        }
    }
}


struct ThirdView: View {
    var body: some View {
        VStack {
            Text("This is the third screen")
                .font(.largeTitle)
            // Add additional content for the third screen here
        }
        .padding()
    }
}


// Modified ContentView with TabView
struct ContentView: View {
    var body: some View {
        TabView {
            MainView()
                .tabItem {
                    Text("Main")
                }
            SecondView()
                .tabItem {
                    Text("Second")
                }
            ThirdView()
                .tabItem {
                    Text("Third")
                }
        }
        .tabViewStyle(PageTabViewStyle())
    }
}
