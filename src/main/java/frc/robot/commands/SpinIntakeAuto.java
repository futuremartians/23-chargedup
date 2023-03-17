package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake;

public class SpinIntakeAuto extends CommandBase {
  private final Intake s_Intake;
  private double intakeSpeed;
  private double timeSpun;
  private double timeToSpin;

  /** Creates a new IntakeJoystick. */
  public SpinIntakeAuto(Intake s_Intake, double intakeSpeed, double timeToSpin) {
    this.s_Intake = s_Intake;
    this.intakeSpeed = intakeSpeed;
    this.timeToSpin = timeToSpin;
    addRequirements(s_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeSpun = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("time spun", timeSpun);
    s_Intake.spinIntake(intakeSpeed);
    timeSpun += 0.02;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Intake.spinIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timeSpun > timeToSpin) {
      return true;
    }
    return false;
  }
}
