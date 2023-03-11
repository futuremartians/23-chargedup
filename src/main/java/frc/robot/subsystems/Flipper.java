// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlipperConstants;

public class Flipper extends SubsystemBase {
  private static Flipper instance;
    public static Flipper getInstance() {
        if (instance == null) instance = new Flipper();
        return instance;
    }

  private WPI_TalonFX flipperMotor;

  /** Creates a new flipper. */
  public Flipper() {
    flipperMotor = new WPI_TalonFX(FlipperConstants.flipperMotorID);
    flipperMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void setFlipperSpeed(double speed) {
    flipperMotor.set(ControlMode.PercentOutput, speed);
}

public void setFlipperVoltage(double voltage) {
    flipperMotor.setVoltage(voltage);
}

public double getMotorPosition() {
return flipperMotor.getSelectedSensorPosition();
}

public double getFlipperMotorVoltage() {
return flipperMotor.getMotorOutputVoltage();
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Flipper Encoder Pos: ", getMotorPosition());
    SmartDashboard.putNumber("flipperMotorVoltage: ", getFlipperMotorVoltage());
  }
}
