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

        var request = URLRequest(url: URL(string: "ws://192.168.50.167:8080/gps")!)
        request.timeoutInterval = 5
        socket = WebSocket(request: request)
        socket.delegate = self  // WebSocketDelegate 설정
        socket.connect()
    }

    func sendMessage(_ message: String) {
        //print("Sending message: \(message)")
        print("message Sended!")
        socket.write(string: message)
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
