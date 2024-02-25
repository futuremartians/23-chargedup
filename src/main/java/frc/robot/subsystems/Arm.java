// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  private static Arm instance;
    public static Arm getInstance() {
        if (instance == null) instance = new Arm();
        return instance;
    }

    private WPI_TalonFX leftArmMotor, rightArmMotor;
    private DutyCycleEncoder armEncoder;

  /** Creates a new Arm. */
  public Arm() {
    leftArmMotor = new WPI_TalonFX(ArmConstants.leftArmMotorID);
    leftArmMotor.setInverted(TalonFXInvertType.Clockwise);
    leftArmMotor.setNeutralMode(NeutralMode.Brake);
    leftArmMotor.set(ControlMode.Follower, ArmConstants.rightArmMotorID);

    rightArmMotor = new WPI_TalonFX(ArmConstants.rightArmMotorID);
    rightArmMotor.setNeutralMode(NeutralMode.Brake);

    armEncoder = new DutyCycleEncoder(0);
    armEncoder.setDistancePerRotation(160714);
    armEncoder.reset();
  }

  public void setArmSpeed(double speed) {
    rightArmMotor.set(ControlMode.PercentOutput, speed);
}

public void setArmVoltage(double voltage) {
    rightArmMotor.setVoltage(voltage);
}

public double getMotorPosition() {
//return rightArmMotor.getSelectedSensorPosition();
return -1 * armEncoder.getDistance();
}

public double getRightMotorVoltage() {
return rightArmMotor.getMotorOutputVoltage();
}

public double getLeftMotorVoltage() {
return leftArmMotor.getMotorOutputVoltage();
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Encoder Pos: ", getMotorPosition());
    SmartDashboard.putNumber("LeftArmVoltage: ", getLeftMotorVoltage());
    SmartDashboard.putNumber("RightArm Voltage: ", getRightMotorVoltage());
  }
}
