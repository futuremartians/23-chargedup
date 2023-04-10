package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;

public class LED extends SubsystemBase {

  private static Spark spark; 

  public LED(int pwmPort) {
    spark = new Spark(pwmPort);

  }


    public void set(double val) {
      if ((val >= -1.0) && (val <= 1.0)) {
        spark.set(val);
      }
    
  }

  public void orange(){

    set(0.65);

  }

 
  
}