// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private static Intake instance;
  public static Intake getInstance() {
      if (instance == null) instance = new Intake();
      return instance;
  }

  private Spark spinRollersMotor;
  /** Creates a new Intake. */
  public Intake() {
    spinRollersMotor = new Spark(20);
  }

  public void spinIntake(double speed) {
    spinRollersMotor.set(speed);
  }

  @Override
  public void periodic() {
    
  }
}
