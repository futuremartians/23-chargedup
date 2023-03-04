// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Elevator;

public class MoveElevatorToPos extends CommandBase {
  private final Elevator s_elevator;
  private final PIDController pidController;

  /** Creates a new ElevatorJoystick. */
  public MoveElevatorToPos(Elevator s_elevator, double pos) {
    this.s_elevator = s_elevator;
    this.pidController = new PIDController(pos, pos, pos);
    pidController.setSetpoint(pos);
    addRequirements(s_elevator);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
