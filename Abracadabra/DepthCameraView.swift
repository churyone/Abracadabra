//
//  DepthCameraView.swift
//  Abracadabra
//
//  Created by 박진우 on 8/12/24.
//

import Foundation
import ARKit
import UIKit

class DepthCameraView: UIView, ARSessionDelegate {
    private let arSession = ARSession()
    private var sceneView: ARSCNView?

    var onDepthDataCaptured: ((Data) -> Void)?

    override init(frame: CGRect) {
        super.init(frame: frame)
        setupDepthCamera()
    }

    required init?(coder: NSCoder) {
        super.init(coder: coder)
        setupDepthCamera()
    }

    private func setupDepthCamera() {
        sceneView = ARSCNView(frame: self.bounds)
        if let sceneView = sceneView {
            self.addSubview(sceneView)
            sceneView.session = arSession
            sceneView.automaticallyUpdatesLighting = true
        }

        let configuration = ARWorldTrackingConfiguration()
        configuration.frameSemantics = .sceneDepth
        arSession.delegate = self
        arSession.run(configuration)
    }

    override func layoutSubviews() {
        super.layoutSubviews()
        sceneView?.frame = self.bounds
    }

    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        guard let sceneDepth = frame.sceneDepth else { return }
        let depthMap = sceneDepth.depthMap

        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }

        let width = CVPixelBufferGetWidth(depthMap)
        let height = CVPixelBufferGetHeight(depthMap)
        let depthData = Data(bytes: CVPixelBufferGetBaseAddress(depthMap)!, count: width * height * MemoryLayout<Float32>.size)

        // 뎁스 데이터가 캡처되면 콜백을 통해 서버로 전송
        onDepthDataCaptured?(depthData)
    }
}
