package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;

public class Camera {
    private static Camera instance;
    public static Camera getInstance() {
        if (instance == null) instance = new Camera();
        return instance;
    }
    
    public void cameraSetup() {
    // Creates UsbCamera and MjpegServer [1] and connects them
    CameraServer.startAutomaticCapture();

    // Creates the CvSink and connects it to the UsbCamera
    CvSink cvSink = CameraServer.getVideo();

    // Creates the CvSource and MjpegServer [2] and connects them
    CvSource outputStream = CameraServer.putVideo("Blur", 640, 480);
    }

}
