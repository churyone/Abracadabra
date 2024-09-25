//
//  WebSocketManager.swift
//  Abracadabra
//
//  Created by 박진우 on 8/13/24.
//

import Foundation
import Starscream
import Combine

class WebSocketManager: NSObject, WebSocketDelegate, ObservableObject {
    var socket: WebSocket!
    @Published var receivedImage: UIImage?
    @Published var receivedText: String?

    override init() {
        super.init()

        var request = URLRequest(url: URL(string: "ws://loe.tosemfdk.org:8080/ws")!)
        request.timeoutInterval = 5
        socket = WebSocket(request: request)
        socket.delegate = self
        socket.connect()
    }

    func sendMessage(_ message: String) {
        socket.write(string: message)
        print("메시지 전송됨!")
    }

    // WebSocketDelegate 메서드
    func didReceive(event: WebSocketEvent, client: WebSocket) {
        switch event {
        case .connected(let headers):
            print("WebSocket 연결됨: \(headers)")
        case .disconnected(let reason, let code):
            print("WebSocket 연결 해제: \(reason) 코드: \(code)")
        case .text(let string):
            DispatchQueue.main.async {
                self.handleReceivedText(string)
            }
        case .binary(let data):
            DispatchQueue.main.async {
                self.handleReceivedData(data)
            }
        case .error(let error):
            print("WebSocket 에러: \(String(describing: error))")
        case .cancelled:
            print("WebSocket 취소됨")
        default:
            break
        }
    }

    private func handleReceivedText(_ string: String) {
        // 수신된 텍스트 처리 (예: JSON 파싱)
        if let data = string.data(using: .utf8),
           let json = try? JSONSerialization.jsonObject(with: data, options: []) as? [String: Any] {
            if let text = json["text"] as? String {
                self.receivedText = text
            }
            if let imageDataString = json["imageData"] as? String,
               let imageData = Data(base64Encoded: imageDataString),
               let image = UIImage(data: imageData) {
                self.receivedImage = image
            }
        } else {
            // 일반 텍스트로 처리
            self.receivedText = string
        }
    }

    private func handleReceivedData(_ data: Data) {
        // 수신된 이진 데이터를 UIImage로 변환
        if let image = UIImage(data: data) {
            self.receivedImage = image
        }
    }
}

