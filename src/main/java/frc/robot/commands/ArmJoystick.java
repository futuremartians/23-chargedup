// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmJoystick extends CommandBase {
  /** Creates a new ArmJoystick. */
  private final Arm s_Arm;
  private DoubleSupplier speed;

  /** Creates a new ArmJoystick. */
  public ArmJoystick(Arm s_Arm, DoubleSupplier speed) {
    this.s_Arm = s_Arm;
    this.speed = speed;
    addRequirements(s_Arm);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedVal = MathUtil.applyDeadband(speed.getAsDouble(), Constants.stickDeadband);
    s_Arm.setArmSpeed(speedVal);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Arm.setArmSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
