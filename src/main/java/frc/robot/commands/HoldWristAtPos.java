package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Wrist;

public class HoldWristAtPos extends CommandBase {
  private final Wrist s_Wrist;
  PIDController pidController;
  private double stallVoltage;
  
  private double pos;

  public HoldWristAtPos(Wrist s_Wrist, double stallVoltage) {
    this.stallVoltage = stallVoltage;
    this.s_Wrist = s_Wrist;
    this.pidController = new PIDController(WristConstants.kp,WristConstants.ki,WristConstants.kd);
    pidController.setSetpoint(WristConstants.holdPos);
    addRequirements(s_Wrist);
    
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double voltage;

    pidController.setTolerance(0);
    voltage = MathUtil.clamp(pidController.calculate(s_Wrist.getMotorPosition()), -stallVoltage, stallVoltage);
    s_Wrist.setWristVoltage(voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Wrist.setWristVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}