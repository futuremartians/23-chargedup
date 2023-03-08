// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Elevator;

public class MoveElevatorToPos extends CommandBase {
  private final Elevator s_elevator;
  PIDController pidController;
  ElevatorFeedforward feedforwardControl;

  public MoveElevatorToPos(Elevator s_elevator, double pos) {
    this.s_elevator = s_elevator;
    this.pidController = new PIDController(0.025,0.0,0.0);
    pidController.setSetpoint(pos);
    feedforwardControl = new ElevatorFeedforward(0, 0.15, 2.83, 0.02);
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
    double feedforward = feedforwardControl.calculate(0);
    double voltage = pidController.calculate(s_elevator.getMotorPosition() + feedforward);
    s_elevator.setElevatorVoltage(voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_elevator.setElevatorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
