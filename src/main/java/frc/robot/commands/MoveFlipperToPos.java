package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Flipper;

public class MoveFlipperToPos extends CommandBase {
  private final Flipper s_Flipper;
  private double pos;
  private double maxVoltage;
  private double stallVoltage;


  /** Creates a new MoveFlipperToPos. */
  public MoveFlipperToPos(Flipper s_Flipper, double pos) {
    this.maxVoltage = maxVoltage;
    this.stallVoltage = stallVoltage;
    this.pos = pos;
    this.s_Flipper = s_Flipper;
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandScheduler.getInstance().schedule(
      new FlipperPID(s_Flipper, pos)
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

    if (Math.abs(s_Flipper.getMotorPosition() - pos) < 1000) {
      atPos = true;
    }
    return atPos;
  }
}
