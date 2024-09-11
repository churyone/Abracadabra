//
//  MotionDataDisplayView.swift
//  Abracadabra
//
//  Created by 박진우 on 8/13/24.
//

import Foundation
import SwiftUI

struct MotionDataDisplayView: View {
    @ObservedObject var motionDataView: MotionDataView

    var body: some View {
        VStack(alignment: .leading) {
            Text("IMU Data")
                .font(.headline)
                .padding(.bottom, 10)
            
            if let accelerometerData = motionDataView.accelerometerData {
                Text("Accelerometer: x=\(accelerometerData.acceleration.x), y=\(accelerometerData.acceleration.y), z=\(accelerometerData.acceleration.z)")
            } else {
                Text("Accelerometer: No Data")
            }

            if let gyroData = motionDataView.gyroData {
                Text("Gyroscope: x=\(gyroData.rotationRate.x), y=\(gyroData.rotationRate.y), z=\(gyroData.rotationRate.z)")
            } else {
                Text("Gyroscope: No Data")
            }

            if let magnetometerData = motionDataView.magnetometerData {
                Text("Magnetometer: x=\(magnetometerData.magneticField.x), y=\(magnetometerData.magneticField.y), z=\(magnetometerData.magneticField.z)")
            } else {
                Text("Magnetometer: No Data")
            }
            // GPS 데이터 (위도, 경도) 표시
            Text("GPS Data")
                .font(.headline)
                .padding(.top, 10)

            if let latitude = motionDataView.latitude, let longitude = motionDataView.longitude {
                Text("Latitude: \(latitude)")
                Text("Longitude: \(longitude)")
            } else {
                Text("GPS: No Data")
            }
        }
        .padding()
    }
}
