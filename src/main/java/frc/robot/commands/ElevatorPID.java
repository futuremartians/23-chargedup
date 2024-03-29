// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class ElevatorPID extends CommandBase {
  private final Elevator s_elevator;
  PIDController pidController;
  ElevatorFeedforward feedforwardControl;
  private double maxVoltage;

  public ElevatorPID(Elevator s_elevator, double pos, double maxVoltage) {
    this.maxVoltage = maxVoltage;
    this.s_elevator = s_elevator;
    this.pidController = new PIDController(ElevatorConstants.kp,ElevatorConstants.ki,ElevatorConstants.kd);
    pidController.setSetpoint(pos);
    
    feedforwardControl = new ElevatorFeedforward(ElevatorConstants.ks, ElevatorConstants.kg, ElevatorConstants.kv, ElevatorConstants.ka);
    addRequirements(s_elevator);
    
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

    if (s_elevator.getMotorPosition() < pidController.getSetpoint()) {
      if (Math.abs(pidController.getPositionError()) > 5000) {
         pidController.setTolerance(5000);
      feedforward = feedforwardControl.calculate(0.5);
      voltage = MathUtil.clamp(pidController.calculate(s_elevator.getMotorPosition() + feedforward), 0, maxVoltage);
      } else {
          pidController.setTolerance(0);
          feedforward = feedforwardControl.calculate(0.1);
         voltage = MathUtil.clamp(pidController.calculate(s_elevator.getMotorPosition()), -0.6, 0.9);
      }
  } else {
    if (Math.abs(pidController.getPositionError()) > 1000) {
      pidController.setTolerance(1000);
      feedforward = feedforwardControl.calculate(-0.3);
      voltage = MathUtil.clamp(pidController.calculate(s_elevator.getMotorPosition() + feedforward), -2.8, 0);
    } else {
      pidController.setTolerance(0);
      feedforward = feedforwardControl.calculate(-0.1);
      voltage = MathUtil.clamp(pidController.calculate(s_elevator.getMotorPosition()), -0.5, 0.9);
    }
  }

    s_elevator.setElevatorVoltage(voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_elevator.setElevatorVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}