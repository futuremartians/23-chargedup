// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class stallIntake extends CommandBase {

  private final Intake s_Intake;
  private BooleanSupplier intakeCone;

  /** Creates a new stallIntake. */
  public stallIntake(Intake s_Intake, BooleanSupplier intakeCone) {
    this.s_Intake = s_Intake;
    this.intakeCone = intakeCone;
    addRequirements(s_Intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intakeCone.getAsBoolean()) {
      s_Intake.spinIntake(0.2);
    } else {
      s_Intake.spinIntake(-0.2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
