//
//  CameraView.swift
//  Abracadabra
//
//  Created by 박진우 on 8/12/24.
//

import Foundation
import SwiftUI
import ARKit

struct CameraView: UIViewControllerRepresentable {
    class Coordinator: NSObject, ARSessionDelegate {
        var parent: CameraView

        init(parent: CameraView) {
            self.parent = parent
        }

        func session(_ session: ARSession, didUpdate frame: ARFrame) {
            // Depth 데이터를 처리하는 코드
            if let depthData = frame.sceneDepth?.depthMap {
                self.parent.depthData = depthData
            }
        }
    }

    @Binding var depthData: CVPixelBuffer?

    func makeCoordinator() -> Coordinator {
        Coordinator(parent: self)
    }

    func makeUIViewController(context: Context) -> UIViewController {
        let viewController = UIViewController()
        let arView = ARSCNView(frame: viewController.view.bounds)
        arView.session.delegate = context.coordinator

        let configuration = ARWorldTrackingConfiguration()
        configuration.frameSemantics = .sceneDepth
        arView.session.run(configuration)

        viewController.view.addSubview(arView)
        return viewController
    }

    func updateUIViewController(_ uiViewController: UIViewController, context: Context) {}
}
