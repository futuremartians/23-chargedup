package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 13;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =  //DONE TODO: This must be tuned to specific robot
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L3);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(22.65); //DONE TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(22.65); //DONE TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //DONE TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(79.98);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(324.4);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //DONE TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(31.64);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(142.73);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    //Elevator Constants
    public static final class ElevatorConstants {
        public static final int leftElevatorMotorID = 14;
        public static final int rightElevatorMotorID = 15;

        public static final int elevatorUpPos = 92500;//Done
        public static final int elevatorDownPos = 0;//Done
        public static final int elevatorCommunityPos = 70000;
        public static final double elevatorDoubleSubstationIntakePos = 34000;
        public static final double elevatorDoubleSubstationIntakePosOld = 17000;

        //PID Controller Values
        public static final double kp = 0.004;
        public static final double ki = 0;
        public static final double kd = 0;

        //FeedForward Values ks, kg, kv, ka
        public static final double ks = 0;
        public static final double kg = 0.15;
        public static final double kv = 2.83;
        public static final double ka = 0.02;
    }

    //Arm Constants
    public static final class ArmConstants {
        public static final int leftArmMotorID = 16;
        public static final int rightArmMotorID = 17;

        public static final double armGroundIntakePos = -45000;//Done
        public static final double armDrivePos = 0;//Done
        public static final double armReadyToFlipPos = -10000;
        public static final double armUnderElevatorPos = -18000;
        public static final double armHighNodeScoringPos = -108000;
        public static final double armMediumNodeScoringPos = -75000;
        public static final double armMediumNodeScoringIntakeSidePos = -20000;
        public static final double armSubstationConeIntakePos = -33000;
        public static final double armDoubleSubstationConIntakePos = -36000;
        public static final double armDoubleSubstationCubeIntakePos = -17000;
        public static final double armDoubleSubstationCubeIntakePosOld = -17000;
        //5500 -35000
        
//-17000, -13000
         //PID Controller Values
         public static final double kp = 0.005;
         public static final double ki = 0;
         public static final double kd = 0;

         //FeedForward Values ks, kg, kv, ka
         public static final double ks = 0;
         public static final double kg = 0.28;
         public static final double kv = 2.69;
         public static final double ka = 0.02;
    }

    public static final class WristConstants {
        public static final int wristMotorID = 18;

        public static double holdPos = 0;

        public static final double wristReadyToFlipPos = -20000;
        public static final double wristDrivePosWhileComingUp = -10000;
        public static final double wristAfterFlipPos = -2000;
        public static final double wristDrivePos = 0;//Done
        public static final double wristUnderElevatorPt1Pos = -43000;
        public static final double wristUnderElevatorPt2Pos = -89000;
        public static final double wristHighNodeScoringPos = -61500;
        public static final double wristFoldedBeforeIntake = 15000;
        public static final double wristIntakeBottomPos = -15400;
        public static final double wristBeforeScoringPos = -80000;
        public static final double wristMediumNodeScoringPos = -84000;
        public static final double wristMediumNodeScoringIntakeSidePos = -22000;
        public static final double wristAutoPreloadPos = -24000;
        public static final double wristSubstationCubeIntakePos = 28800;
        public static final double wristSubstationConeIntakePos = 4750;
        public static final double wristMissingPolePos = -84000;
        public static final double wristCommunityPos = -60000;
        public static final double wristDoubleSubstationConeIntakePos = 18250;
        public static final double wristDoubleSubstationCubeIntakePos = -33000;
        public static final double wristDoubleSubstationCubeIntakePosOld = -22000;

         //PID Controller Values
         public static final double kp = 0.01;//old val: 0.01
         public static final double ki = 0;
         public static final double kd = 0;
    }

    public static final class FlipperConstants {
        public static final int flipperMotorID = 19;

        public static final double flipperDrivePos = 0;
        public static final double flipperScoringPos = -25750;

         //PID Controller Values
         public static final double kp = 0.022;
         public static final double ki = 0.00;
         public static final double kd = 0;
    }
    public static final class PWMPorts {
        public static final int kBlinkin = 0;
    }
}
