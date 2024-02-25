// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import java.beans.Encoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;

public class Wrist extends SubsystemBase {

  private static Wrist instance;
    public static Wrist getInstance() {
        if (instance == null) instance = new Wrist();
        return instance;
    }

  private WPI_TalonFX wristMotor;
  private DutyCycleEncoder wristEncoder;
  

  /** Creates a new Wrist. */
  public Wrist() {
    wristMotor = new WPI_TalonFX(WristConstants.wristMotorID);
    wristMotor.setNeutralMode(NeutralMode.Brake);
    wristEncoder = new DutyCycleEncoder(1);
    wristEncoder.setDistancePerRotation(208620);
    wristEncoder.reset();
  }

  public void setWristSpeed(double speed) {
    wristMotor.set(ControlMode.PercentOutput, speed);
}

public void setWristVoltage(double voltage) {
    wristMotor.setVoltage(voltage);
}

public double getMotorPosition() {
  return wristMotor.getSelectedSensorPosition();
  //return -1*wristEncoder.getDistance();
}

public double getWristMotorVoltage() {
return wristMotor.getMotorOutputVoltage();
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Encoder Pos: ", getMotorPosition());
    SmartDashboard.putNumber("WristMotorVoltage: ", getWristMotorVoltage());
  }
}
