//
//  Item.swift
//  Abracadabra
//
//  Created by 박진우 on 8/12/24.
//

import Foundation
import SwiftData

@Model
final class Item {
    var timestamp: Date
    
    init(timestamp: Date) {
        self.timestamp = timestamp
    }
}
