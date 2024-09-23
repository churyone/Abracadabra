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
    @Binding var cameraIntrinsics: (fx: Float, fy: Float, cx: Float, cy: Float)?  // cameraIntrinsics 추가

    class Coordinator: NSObject, ARSessionDelegate {
        var parent: ARDepthCameraView

        init(parent: ARDepthCameraView) {
            self.parent = parent
        }

        func session(_ session: ARSession, didUpdate frame: ARFrame) {
            let rgbPixelBuffer = frame.capturedImage
            let rgbWidth = CVPixelBufferGetWidth(rgbPixelBuffer)
            let rgbHeight = CVPixelBufferGetHeight(rgbPixelBuffer)
            print("RGB 카메라 해상도: \(rgbWidth) x \(rgbHeight)")
            if let sceneDepth = frame.sceneDepth {
                let depthMap = sceneDepth.depthMap
                // 카메라 파라미터 가져오기
                let intrinsics = frame.camera.intrinsics
                print("Camera Intrinsics: \(intrinsics)")
                let fx = intrinsics[0, 0]  // 초점 거리 (focal length) x
                let fy = intrinsics[1, 1]  // 초점 거리 (focal length) y
                let cx = intrinsics[2, 0]  // 주점 x (principal point)
                let cy = intrinsics[2, 1]  // 주점 y (principal point)

                DispatchQueue.main.async {
                    // Depth 데이터를 카메라 뷰와 동일한 비율로 조정
                    self.parent.depthData = self.scaleAndRotatePixelBuffer(depthMap, targetSize: CGSize(width: UIScreen.main.bounds.width, height: 300))
                    self.parent.cameraIntrinsics = (fx, fy, cx, cy)

                }
            }
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
        configuration.frameSemantics = .sceneDepth
        view.session.run(configuration)
        return view
    }

    func updateUIView(_ uiView: ARSCNView, context: Context) {}
}
