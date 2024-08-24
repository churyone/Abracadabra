//
//  MotionDataView.swift
//  Abracadabra
//
//  Created by 박진우 on 8/13/24.
//

import Foundation
import SwiftUI
import CoreMotion

class MotionDataView: ObservableObject {  // ObservableObject 준수
    private var motionManager = CMMotionManager()
    
    @Published var accelerometerData: CMAccelerometerData?
    @Published var gyroData: CMGyroData?
    @Published var magnetometerData: CMMagnetometerData?
    
    init() {
        startMotionUpdates()
    }
    
    private func startMotionUpdates() {
        if motionManager.isAccelerometerAvailable {
            motionManager.accelerometerUpdateInterval = 0.1  // 10 Hz
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
}
