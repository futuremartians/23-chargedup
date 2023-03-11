package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase {
    private static Limelight instance;
    public static Limelight getInstance() {
        if (instance == null) instance = new Limelight();
        return instance;
    }

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTable cameraPublisherTable = NetworkTableInstance.getDefault().getTable("CameraPublisher").getSubTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    void initalize() {
        setCamMode();
        setIPDetails();
    }

    public void setCamMode() {
        table.getEntry("camMode").setNumber(1);
    }

    public void setIPDetails() {
        cameraPublisherTable.getEntry("source").setValue("ip:10.90.23.11:5800");
        cameraPublisherTable.getEntry("streams").setValue("ip:10.90.23.11:5800");
    }
    
}
