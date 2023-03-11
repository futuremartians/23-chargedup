// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Wrist;

public class WristJoystick extends CommandBase {
  /** Creates a new WristJoystick. */
  private final Wrist s_Wrist;
  private DoubleSupplier speed;

  /** Creates a new WristJoystick. */
  public WristJoystick(Wrist s_Wrist, DoubleSupplier speed) {
    this.s_Wrist = s_Wrist;
    this.speed = speed;
    addRequirements(s_Wrist);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedVal = MathUtil.applyDeadband(speed.getAsDouble(), Constants.stickDeadband);
    s_Wrist.setWristSpeed(speedVal);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Wrist.setWristSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
