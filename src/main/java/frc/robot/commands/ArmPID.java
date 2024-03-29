package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class ArmPID extends CommandBase {
  private final Arm s_Arm;
  PIDController pidController;
  ArmFeedforward feedforwardControl;
  private double pos;
  private double maxVoltage;

  public ArmPID(Arm s_Arm, double pos, double maxVoltage) {
    this.pos = pos;
    this.maxVoltage = maxVoltage;
    this.s_Arm = s_Arm;
    this.pidController = new PIDController(ArmConstants.kp,ArmConstants.ki,ArmConstants.kd);
    pidController.setSetpoint(pos);
    
    feedforwardControl = new ArmFeedforward(ArmConstants.ks, ArmConstants.kg, ArmConstants.kv, ArmConstants.ka);
    addRequirements(s_Arm);
    
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double feedforward;
    double voltage;

      if (Math.abs(pidController.getPositionError()) > 5000) {
         pidController.setTolerance(5000);
      feedforward = feedforwardControl.calculate(pos, 0.4, 0.5);
      voltage = MathUtil.clamp(pidController.calculate(s_Arm.getMotorPosition()+ feedforward), -maxVoltage, maxVoltage);
      } else {
          pidController.setTolerance(0);
          //feedforward = feedforwardControl.calculate(pos, 0.2, 0.2);
         voltage = MathUtil.clamp(pidController.calculate(s_Arm.getMotorPosition()), -0.9, 0.9);
      }
    s_Arm.setArmVoltage(voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
