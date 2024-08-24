# Uncomment the next line to define a global platform for your project
platform :ios, '17.0'

target 'Abracadabra' do
  # Comment the next line if you don't want to use dynamic frameworks
  use_frameworks!

  # Pods for Abracadabra
  pod 'Starscream', '= 4.0.4'
end
post_install do |installer|
  installer.generated_projects.each do |project|
      project.targets.each do |target|
          target.build_configurations.each do |config|
              config.build_settings['IPHONEOS_DEPLOYMENT_TARGET'] = '17.0'
          end
      end
  end
end