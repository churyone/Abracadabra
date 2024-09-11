//
//  WebSocketManager.swift
//  Abracadabra
//
//  Created by 박진우 on 8/13/24.
//

import Foundation
import Starscream

class WebSocketManager: NSObject, WebSocketDelegate {
    var socket: WebSocket!

    override init() {
        super.init()

        var request = URLRequest(url: URL(string: "ws://loe.tosemfdk.org:8080/ws")!)
        request.timeoutInterval = 5
        socket = WebSocket(request: request)
        socket.delegate = self  // WebSocketDelegate 설정
        socket.connect()
    }

    func sendMessage(_ message: String) {
        socket.write(string: message)
        print("message Sended!")
    }

    // WebSocketDelegate methods
    func didReceive(event: WebSocketEvent, client: WebSocket) {
        switch event {
        case .connected(let headers):
            print("WebSocket connected: \(headers)")
        case .disconnected(let reason, let code):
            print("WebSocket disconnected: \(reason) with code: \(code)")
        case .text(let string):
            print("Received text: \(string)")
        case .binary(let data):
            print("Received data: \(data.count) bytes")
        case .error(let error):
            print("WebSocket error: \(String(describing: error))")
        case .cancelled:
            print("WebSocket cancelled")
        case .ping(_):
            break
        case .pong(_):
            break
        case .viabilityChanged(_):
            break
        case .reconnectSuggested(_):
            break
        }
    }
}
