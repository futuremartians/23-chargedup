// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class MoveArmToPos extends CommandBase {
  private final Arm s_Arm;
  PIDController pidController;
  ArmFeedforward feedforwardControl;
  private double pos;

  public MoveArmToPos(Arm s_Arm, double pos) {
    this.pos = pos;
    this.s_Arm = s_Arm;
    this.pidController = new PIDController(ArmConstants.kp,ArmConstants.ki,ArmConstants.kd);
    pidController.setSetpoint(pos);
    
    feedforwardControl = new ArmFeedforward(0, 0.15, 2.83, 0.02);
    addRequirements(s_Arm);
    
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double feedforward;
    double voltage;

    if (s_Arm.getMotorPosition() < pidController.getSetpoint()) {
      if (Math.abs(pidController.getPositionError()) > 1000) {
         pidController.setTolerance(1000);
      feedforward = feedforwardControl.calculate(pos, 0.4, 0.5);
      voltage = MathUtil.clamp(pidController.calculate(s_Arm.getMotorPosition()+ feedforward), 0, 3.5);
      } else {
          pidController.setTolerance(0);
          feedforward = feedforwardControl.calculate(pos, 0.2, 0.3);
         voltage = MathUtil.clamp(pidController.calculate(s_Arm.getMotorPosition()), -0.5, 0.9);
      }
  } else {
    if (Math.abs(pidController.getPositionError()) > 1000) {
      pidController.setTolerance(1000);
      feedforward = feedforwardControl.calculate(-0.3);
      voltage = MathUtil.clamp(pidController.calculate(s_Arm.getMotorPosition() + feedforward), -1.5, 0);
    } else {
      pidController.setTolerance(0);
      feedforward = feedforwardControl.calculate(-0.1);
      voltage = MathUtil.clamp(pidController.calculate(s_Arm.getMotorPosition()), -0.5, 0.9);
    }
  }

    s_Arm.setArmVoltage(voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Arm.setArmVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

