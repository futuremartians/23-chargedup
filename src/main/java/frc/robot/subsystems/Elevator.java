package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
public class Elevator extends SubsystemBase {
    private static Elevator instance;
    public static Elevator getInstance() {
        if (instance == null) instance = new Elevator();
        return instance;
    }

    private WPI_TalonFX leftElevatorMotor, rightElevatorMotor;

    public Elevator() {
        leftElevatorMotor = new WPI_TalonFX(ElevatorConstants.leftElevatorMotorID);
        leftElevatorMotor.setNeutralMode(NeutralMode.Brake);
        leftElevatorMotor.set(ControlMode.Follower, ElevatorConstants.rightElevatorMotorID);

        rightElevatorMotor = new WPI_TalonFX(ElevatorConstants.rightElevatorMotorID);
        rightElevatorMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void setElevatorSpeed(double speed) {
        rightElevatorMotor.set(ControlMode.PercentOutput, speed);
    }

    public void setElevatorVoltage(double voltage) {
        rightElevatorMotor.setVoltage(voltage);
    }

    public double getMotorPosition() {
		return rightElevatorMotor.getSelectedSensorPosition();
	}

    public double getRightMotorVoltage() {
		return rightElevatorMotor.getMotorOutputVoltage();
	}

    public double getLeftMotorVoltage() {
		return leftElevatorMotor.getMotorOutputVoltage();
	}

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Encoder Pos: ", getMotorPosition());
        SmartDashboard.putNumber("LeftElevatorVoltage: ", getLeftMotorVoltage());
        SmartDashboard.putNumber("RightElevator Voltage: ", getRightMotorVoltage());
    }
}
