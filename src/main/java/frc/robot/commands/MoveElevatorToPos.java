// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class MoveElevatorToPos extends CommandBase {
  private final Elevator s_Elevator;
  private double pos;
  private double maxVoltage;

  /** Creates a new MoveElevatorToPos. */
  public MoveElevatorToPos(Elevator s_Elevator, double pos) {
    this.maxVoltage = maxVoltage;
    this.pos = pos;
    this.s_Elevator = s_Elevator;
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandScheduler.getInstance().schedule(
      new ElevatorPID(s_Elevator, pos)
      );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean atPos = false;

    if (Math.abs(s_Elevator.getMotorPosition() - pos) < 2000) {
      atPos = true;
    }
    return atPos;
  }
}

