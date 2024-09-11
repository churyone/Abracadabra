//
//  MotionDataView.swift
//  Abracadabra
//
//  Created by 박진우 on 8/13/24.
//

import Foundation
import SwiftUI
import CoreMotion
import CoreLocation


class MotionDataView: NSObject, ObservableObject, CLLocationManagerDelegate {  // ObservableObject 준수
    private var motionManager = CMMotionManager()
    private var locationManager = CLLocationManager()
    
    @Published var accelerometerData: CMAccelerometerData?
    @Published var gyroData: CMGyroData?
    @Published var magnetometerData: CMMagnetometerData?
    @Published var latitude: Double?  // 위도
    @Published var longitude: Double? // 경도
    
    override init() {
        super.init()
        
        locationManager.delegate = self
        locationManager.requestWhenInUseAuthorization()  // 위치 권한 요청
        locationManager.startUpdatingLocation()  // 위치 업데이트 시작
        
        startMotionUpdates()  // IMU 데이터 업데이트
    }
    
    
    private func startMotionUpdates() {
        if motionManager.isAccelerometerAvailable {
            motionManager.accelerometerUpdateInterval = 0.1  // 0.1초마다(10Hz) 데이터 업데이트
            motionManager.startAccelerometerUpdates(to: .main) { data, error in
                guard let data = data, error == nil else { return }
                self.accelerometerData = data
            }
        }

        
        if motionManager.isGyroAvailable {
            motionManager.gyroUpdateInterval = 0.1  // 10 Hz
            motionManager.startGyroUpdates(to: .main) { data, error in
                guard let data = data, error == nil else { return }
                self.gyroData = data
            }
        }
        
        if motionManager.isMagnetometerAvailable {
            motionManager.magnetometerUpdateInterval = 0.1  // 10 Hz
            motionManager.startMagnetometerUpdates(to: .main) { data, error in
                guard let data = data, error == nil else { return }
                self.magnetometerData = data
            }
        }
    }

    func stopMotionUpdates() {
        motionManager.stopAccelerometerUpdates()
        motionManager.stopGyroUpdates()
        motionManager.stopMagnetometerUpdates()
    }
    
    // CLLocationManagerDelegate 메서드
    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        guard let location = locations.last else { return }
        self.latitude = location.coordinate.latitude  // 위도 저장
        self.longitude = location.coordinate.longitude  // 경도 저장
    }
}
