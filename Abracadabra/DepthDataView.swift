//
//  DepthDataView.swift
//  Abracadabra
//
//  Created by 박진우 on 8/12/24.
//

import Foundation
import SwiftUI
import CoreImage
import CoreImage.CIFilterBuiltins

struct DepthDataView: View {
    var depthData: CVPixelBuffer?

    var body: some View {
        if let depthData = depthData, let uiImage = convertToColorMappedUIImage(pixelBuffer: depthData) {
            Image(uiImage: uiImage)
                .resizable()
                .scaledToFit()
        } else {
            Text("Depth data not available")
        }
    }
    
    func convertToColorMappedUIImage(pixelBuffer: CVPixelBuffer) -> UIImage? {
        let ciImage = CIImage(cvPixelBuffer: pixelBuffer)

        // Normalize the depth data to a 0-1 range
        let normalizeFilter = CIFilter(name: "CIColorControls")!
        normalizeFilter.setValue(ciImage, forKey: kCIInputImageKey)
        normalizeFilter.setValue(1.0, forKey: kCIInputContrastKey)
        normalizeFilter.setValue(0.0, forKey: kCIInputBrightnessKey)
        normalizeFilter.setValue(1.0, forKey: kCIInputSaturationKey)
        guard let normalizedImage = normalizeFilter.outputImage else { return nil }

        // Apply a false color filter to map depth data to colors
        let falseColorFilter = CIFilter.falseColor()
        falseColorFilter.inputImage = normalizedImage
        falseColorFilter.color0 = CIColor(red: 1, green: 0, blue: 0) // Red for close objects
        falseColorFilter.color1 = CIColor(red: 0, green: 0, blue: 1) // Blue for far objects

        guard let outputImage = falseColorFilter.outputImage else { return nil }

        let context = CIContext()
        if let cgImage = context.createCGImage(outputImage, from: outputImage.extent) {
            return UIImage(cgImage: cgImage)
        }
        return nil
    }
}
