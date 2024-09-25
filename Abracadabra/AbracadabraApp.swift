//
//  AbracadabraApp.swift
//  Abracadabra
//
//  Created by 박진우 on 8/12/24.
//

import SwiftUI
import SwiftData

@main
struct AbracadabraApp: App {
    var sharedModelContainer: ModelContainer = {
        let schema = Schema([
            Item.self,
        ])
        let modelConfiguration = ModelConfiguration(schema: schema, isStoredInMemoryOnly: false)

        do {
            return try ModelContainer(for: schema, configurations: [modelConfiguration])
        } catch {
            fatalError("Could not create ModelContainer: \(error)")
        }
    }()

    @StateObject var webSocketManager = WebSocketManager()

    var body: some Scene {
        WindowGroup {
            ContentView()
                // ContentView에 webSocketManager를 environmentObject로 주입합니다.
                .environmentObject(webSocketManager)
        }
        .modelContainer(sharedModelContainer)
    }
}
