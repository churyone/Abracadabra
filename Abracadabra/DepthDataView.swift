import SwiftUI
import CoreImage
import CoreImage.CIFilterBuiltins
import ARKit


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

        // Apply a color cube filter to map depth data to Blue -> Green -> Red
        guard let colorCubeFilter = CIFilter(name: "CIColorCube") else { return nil }

        // Define the dimension for the color cube
        let dimension = 64
        var cubeData = [Float](repeating: 0, count: dimension * dimension * dimension * 4)

        for z in 0..<dimension {
            for y in 0..<dimension {
                for x in 0..<dimension {
                    let offset = 4 * (z * dimension * dimension + y * dimension + x)
                    
                    let value = Float(x) / Float(dimension - 1)  // Normalized value between 0 and 1
                    
                    // Map from blue (close) to green (middle) to red (far)
                    let color = colorForValue(value: value)
                    cubeData[offset + 0] = color.red   // Red component
                    cubeData[offset + 1] = color.green // Green component
                    cubeData[offset + 2] = color.blue  // Blue component
                    cubeData[offset + 3] = 1.0         // Alpha component (fully opaque)
                }
            }
        }

        let dataSize = cubeData.count * MemoryLayout<Float>.size
        let cubeDataPointer = UnsafeMutablePointer<Float>.allocate(capacity: cubeData.count)
        cubeDataPointer.initialize(from: cubeData, count: cubeData.count)

        let cubeDataBuffer = NSData(bytesNoCopy: cubeDataPointer, length: dataSize, freeWhenDone: true)
        
        colorCubeFilter.setValue(cubeDataBuffer, forKey: "inputCubeData")
        colorCubeFilter.setValue(dimension, forKey: "inputCubeDimension")
        colorCubeFilter.setValue(normalizedImage, forKey: kCIInputImageKey)

        guard let outputImage = colorCubeFilter.outputImage else { return nil }

        let context = CIContext()
        if let cgImage = context.createCGImage(outputImage, from: outputImage.extent) {
            return UIImage(cgImage: cgImage)
        }

        return nil
    }

    // Function to return a color for a given normalized depth value (0.0 to 1.0)
    func colorForValue(value: Float) -> (red: Float, green: Float, blue: Float) {
        if value < 0.5 {
            // Close: Transition from blue (close) to green (middle)
            let ratio = value / 0.5
            return (red: 0.0, green: ratio, blue: 1.0 - ratio)
        } else {
            // Far: Transition from green (middle) to red (far)
            let ratio = (value - 0.5) / 0.5
            return (red: ratio, green: 1.0 - ratio, blue: 0.0)
        }
    }
}


