// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Wrist;

public class MoveWristToPos extends CommandBase {
  private final Wrist s_Wrist;
  PIDController pidController;
  private double maxVoltage;
  private double stallVoltage;
  
  private double pos;

  public MoveWristToPos(Wrist s_Wrist, double pos, double maxVoltage, double stallVoltage) {
    this.maxVoltage = maxVoltage;
    this.stallVoltage = stallVoltage;
    this.pos = pos;
    this.s_Wrist = s_Wrist;
    this.pidController = new PIDController(WristConstants.kp,WristConstants.ki,WristConstants.kd);
    pidController.setSetpoint(pos);
    addRequirements(s_Wrist);
    
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

      if (Math.abs(pidController.getPositionError()) > 1000) {
         pidController.setTolerance(1000);
         voltage = MathUtil.clamp(pidController.calculate(s_Wrist.getMotorPosition()), -maxVoltage, maxVoltage);
      } else {
          pidController.setTolerance(0);
          voltage = MathUtil.clamp(pidController.calculate(s_Wrist.getMotorPosition()), -stallVoltage, stallVoltage);
      }
    s_Wrist.setWristVoltage(voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}