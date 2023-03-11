// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Flipper;

public class FlipperJoystick extends CommandBase {
  /** Creates a new FlipperJoystick. */
  private final Flipper s_Flipper;
  private DoubleSupplier speed;

  /** Creates a new FlipperJoystick. */
  public FlipperJoystick(Flipper s_Flipper, DoubleSupplier speed) {
    this.s_Flipper = s_Flipper;
    this.speed = speed;
    addRequirements(s_Flipper);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedVal = MathUtil.applyDeadband(speed.getAsDouble(), Constants.stickDeadband);
    s_Flipper.setFlipperSpeed(speedVal);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Flipper.setFlipperSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
