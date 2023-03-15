// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class Intake extends SubsystemBase {

  private static Intake instance;
  public static Intake getInstance() {
      if (instance == null) instance = new Intake();
      return instance;
  }

  private CANSparkMax spinRollersMotor;
  /** Creates a new Intake. */
  public Intake() {
    spinRollersMotor = new CANSparkMax(20, CANSparkMaxLowLevel.MotorType.kBrushed);
  }

  public void spinIntake(double speed) {
    spinRollersMotor.set(speed);
  }

  public double intakeSpeed(boolean coneIntake) {
    if (coneIntake) {
      return 0.3;
    } else {
      return -0.3;
    }
  }

  @Override
  public void periodic() {
    
  }
}
