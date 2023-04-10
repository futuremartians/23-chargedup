// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class SingleSubIntake extends CommandBase {

  private final Wrist s_Wrist;
  private final Arm s_Arm;
  boolean coneMode;

  /** Creates a new SingleSubIntake. */
  public SingleSubIntake(Arm s_Arm, Wrist s_Wrist, boolean coneMode) {
    this.s_Wrist = s_Wrist;
    this.s_Arm = s_Arm;
    this.coneMode = coneMode;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (coneMode) {
      Commands.parallel(
        new MoveArmToPos(s_Arm, ArmConstants.armSubstationConeIntakePos, 2),
        Commands.sequence(
            Commands.waitSeconds(0.3),
            new MoveWristToPos(s_Wrist, WristConstants.wristSubstationConeIntakePos, 1.8, 0.8)
        )
    );
    } else {
      Commands.parallel(
        new MoveArmToPos(s_Arm, ArmConstants.armGroundIntakePos, 2),
        Commands.sequence(
            Commands.waitSeconds(0.3),
            new MoveWristToPos(s_Wrist, WristConstants.wristSubstationCubeIntakePos, 1.8, 0.8)
        )
    );

    }
  }

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
