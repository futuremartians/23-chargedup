// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FlipperConstants;
import frc.robot.subsystems.Flipper;

public class FlipperPID extends CommandBase {
  private final Flipper s_Flipper;
  PIDController pidController;
  
  private double pos;

  public FlipperPID(Flipper s_Flipper, double pos) {
    this.pos = pos;
    this.s_Flipper = s_Flipper;
    this.pidController = new PIDController(FlipperConstants.kp,FlipperConstants.ki,FlipperConstants.kd);
    pidController.setSetpoint(pos);
    addRequirements(s_Flipper);
    
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

      if (Math.abs(pidController.getPositionError()) > 5000) {
         pidController.setTolerance(5000);

         voltage = MathUtil.clamp(pidController.calculate(s_Flipper.getMotorPosition()), -2.5, 2.5);
      } else {
          pidController.setTolerance(0);
          voltage = MathUtil.clamp(pidController.calculate(s_Flipper.getMotorPosition()), -1, 1);
      }
    s_Flipper.setFlipperVoltage(voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Flipper.setFlipperVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}