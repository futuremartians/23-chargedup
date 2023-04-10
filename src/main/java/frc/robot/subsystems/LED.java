package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class LED extends SubsystemBase {

  /* Rev Robotics Blinkin takes a PWM signal from 1000-2000us
   * This is identical to a SparkMax motor. 
   *  -1  corresponds to 1000us
   *  0   corresponds to 1500us
   *  +1  corresponds to 2000us
   */
  private Spark spark = new Spark(0);

  /**
   * Creates a new Blinkin LED controller.
   * 
   *   The PWM port the Blinkin is connected to.
   */
  public LED(int pwmPort) {
    Spark m_led = new Spark(pwmPort);

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