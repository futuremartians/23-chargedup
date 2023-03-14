// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Wrist;

public class MoveWristToPos extends CommandBase {
  private final Wrist s_Wrist;
  private double pos;
  private double maxVoltage;
  private double stallVoltage;


  /** Creates a new MoveWristToPos. */
  public MoveWristToPos(Wrist s_Wrist, double pos, double maxVoltage, double stallVoltage) {
    this.maxVoltage = maxVoltage;
    this.stallVoltage = stallVoltage;
    this.pos = pos;
    this.s_Wrist = s_Wrist;
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandScheduler.getInstance().schedule(
      new WristPID(s_Wrist, pos, maxVoltage, stallVoltage)
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

    if (Math.abs(s_Wrist.getMotorPosition() - pos) < 1000) {
      atPos = true;
    }
    return atPos;
  }
}
