//
//  ARDepthCameraView.swift
//  Abracadabra
//
//  Created by 박진우 on 8/12/24.
//

import Foundation
import ARKit
import SwiftUI

struct ARDepthCameraView: UIViewRepresentable {
    @Binding var depthData: CVPixelBuffer?
    @Binding var rgbImage: CIImage?
    @Binding var cameraIntrinsics: (fx: Float, fy: Float, cx: Float, cy: Float)?  // cameraIntrinsics 추가

    class Coordinator: NSObject, ARSessionDelegate {
        var parent: ARDepthCameraView

        init(parent: ARDepthCameraView) {
            self.parent = parent
        }

        func session(_ session: ARSession, didUpdate frame: ARFrame) {
            DispatchQueue.global(qos: .userInitiated).async {
                // RGB 이미지 처리
                let rgbPixelBuffer = frame.capturedImage
                let rgbCIImage = CIImage(cvPixelBuffer: rgbPixelBuffer)
                
                // 깊이 데이터 처리
                var depthDataBuffer: CVPixelBuffer?
                var intrinsics: simd_float3x3?
                if let sceneDepth = frame.sceneDepth {
                    depthDataBuffer = sceneDepth.depthMap
                    intrinsics = frame.camera.intrinsics
                } else {
                    print("Scene depth is not available")
                }
                
                // UI 업데이트는 메인 스레드에서
                DispatchQueue.main.async {
                    self.parent.rgbImage = rgbCIImage
                    if let depthDataBuffer = depthDataBuffer, let intrinsics = intrinsics {
                        if let scaledDepthData = self.scaleAndRotatePixelBuffer(depthDataBuffer, targetSize: CGSize(width: UIScreen.main.bounds.width, height: 300)) {
                            self.parent.depthData = scaledDepthData
                            let fx = intrinsics[0, 0]
                            let fy = intrinsics[1, 1]
                            let cx = intrinsics[2, 0]
                            let cy = intrinsics[2, 1]
                            self.parent.cameraIntrinsics = (fx, fy, cx, cy)
                        } else {
                            print("Failed to scale and rotate depth data")
                        }
                    } else {
                        print("Depth data or intrinsics not available")
                    }
                }
            }
        }
        
        func session(_ session: ARSession, didFailWithError error: Error) {
            print("ARSession failed with error: \(error.localizedDescription)")
        }
        
        func sessionWasInterrupted(_ session: ARSession) {
            print("ARSession was interrupted")
        }
        
        func sessionInterruptionEnded(_ session: ARSession) {
            print("ARSession interruption ended")
        }
        
        // 다시 PixelBuffer로 변환
        func scaleAndRotatePixelBuffer(_ pixelBuffer: CVPixelBuffer, targetSize: CGSize) -> CVPixelBuffer? {
            let ciImage = CIImage(cvPixelBuffer: pixelBuffer)

            // 이미지 회전 - 90도 시계방향
            let rotatedCIImage = ciImage.oriented(.right)

            // 이미지 크기 조정 - 대상 사이즈에 맞게 스케일링
            let scaleX = targetSize.width / rotatedCIImage.extent.height // 가로 확장 비율
            let scaleY = targetSize.height / rotatedCIImage.extent.width // 세로 축소 비율

            let scaledCIImage = rotatedCIImage.transformed(by: CGAffineTransform(scaleX: scaleX, y: scaleY))

            let context = CIContext()
            var scaledPixelBuffer: CVPixelBuffer?
            let status = CVPixelBufferCreate(kCFAllocatorDefault,
                                             Int(scaledCIImage.extent.width),
                                             Int(scaledCIImage.extent.height),
                                             CVPixelBufferGetPixelFormatType(pixelBuffer),
                                             nil,
                                             &scaledPixelBuffer)

            if status == kCVReturnSuccess, let scaledPixelBuffer = scaledPixelBuffer {
                context.render(scaledCIImage, to: scaledPixelBuffer)
                return scaledPixelBuffer
            } else {
                return nil
            }
        }
    }

    func makeCoordinator() -> Coordinator {
        Coordinator(parent: self)
    }

    func makeUIView(context: Context) -> ARSCNView {
        let view = ARSCNView()
        view.session.delegate = context.coordinator
        let configuration = ARWorldTrackingConfiguration()
        
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
            configuration.frameSemantics.insert(.sceneDepth)
        } else {
            print("Scene depth is not supported on this device")
        }
        
        view.session.run(configuration)
        return view
    }

    func updateUIView(_ uiView: ARSCNView, context: Context) {}
}
