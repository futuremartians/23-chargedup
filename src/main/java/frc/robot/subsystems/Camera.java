package frc.robot.subsystems;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;

public class Camera {
    private static Camera instance;
    public static Camera getInstance() {
        if (instance == null) instance = new Camera();
        return instance;
    }

    // Creates UsbCamera and MjpegServer [1] and connects them
    UsbCamera usbCamera = new UsbCamera("Front Camera", 0);
    MjpegServer mjpegServer1 = new MjpegServer("serve_USB Camera 0", 1181);

    // Creates the CvSink and connects it to the UsbCamera
    CvSink cvSink = new CvSink("opencv_USB Camera 0");
    
    // Creates the CvSource and MjpegServer [2] and connects them
    CvSource outputStream = new CvSource("Blur", PixelFormat.kMJPEG, 640, 480, 30);
    MjpegServer mjpegServer2 = new MjpegServer("serve_Blur", 1182);
    
    public void cameraSetup() {
        mjpegServer1.setSource(usbCamera);
        cvSink.setSource(usbCamera);
        mjpegServer2.setSource(outputStream);
    }

}
