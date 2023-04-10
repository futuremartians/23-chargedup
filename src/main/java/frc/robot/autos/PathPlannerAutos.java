package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FlipperConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class PathPlannerAutos {
   private static Intake s_Intake = Intake.getInstance();
   private static Arm s_Arm = Arm.getInstance();
   private static Swerve s_Swerve = Swerve.getInstance();
   private static Wrist s_Wrist = Wrist.getInstance();
   private static Elevator s_Elevator = Elevator.getInstance();
   private static Flipper s_Flipper = Flipper.getInstance();
   static HashMap<String, Command> eventMap = new HashMap<String, Command>();

    private static Command lastCommand;
    private static Command chosenAuto;
    //private SwerveAutoBuilder autoBuilder;

    public final static Command scorePreloadCube = 
    Commands.sequence(
    Commands.parallel(
       new SpinIntakeAuto(s_Intake, -0.9, 0.2),
       Commands.sequence(
       Commands.parallel(
            new MoveArmToPos(s_Arm, ArmConstants.armDrivePos, 1),
            new MoveWristToPos(s_Wrist, WristConstants.wristAutoPreloadPos, 1.75, 0.8)
         )
       )
    ),
    Commands.sequence(
      new SpinIntakeAuto(s_Intake, 1, 0.8),
      new MoveWristToPos(s_Wrist, WristConstants.wristDrivePos, 1.4, 1)
    )  
    );

    public final static Command scorePreloadCone =
    Commands.sequence(
        new SpinIntakeAuto(s_Intake, 1, 0.4),
        Commands.sequence(
        Commands.sequence(
            new MoveArmToPos(s_Arm, ArmConstants.armDrivePos, 1.5),
            new MoveWristToPos(s_Wrist, WristConstants.wristReadyToFlipPos, 2, 1.2),
            new MoveFlipperToPos(s_Flipper, FlipperConstants.flipperScoringPos)
           // new MoveWristToPos(s_Wrist, WristConstants.wristAfterFlipPos, 2, 1.2)
        ),
    Commands.parallel(
        new MoveElevatorToPos(s_Elevator, ElevatorConstants.elevatorUpPos),
        Commands.sequence(
            Commands.waitSeconds(0.2), 
            new MoveWristToPos(s_Wrist, WristConstants.wristUnderElevatorPt1Pos, 3.2, 1),
            Commands.parallel(
                new MoveWristToPos(s_Wrist, WristConstants.wristUnderElevatorPt2Pos, 3.5, 1),
                new MoveArmToPos(s_Arm, ArmConstants.armHighNodeScoringPos, 3.3),
                Commands.sequence(
                    Commands.waitSeconds(0.4),
                    Commands.parallel(
                        new MoveArmToPos(s_Arm, ArmConstants.armHighNodeScoringPos, 6),
                        Commands.sequence(
                            Commands.waitSeconds(0.5),
                            new MoveWristToPos(s_Wrist, WristConstants.wristHighNodeScoringPos, 3.3, 0.9)
                            )
                        )
                    )
                )
            )
        )
    ),
 new SpinIntakeAuto(s_Intake, -1, 0.6),
    Commands.sequence(
        Commands.parallel(
            new MoveWristToPos(s_Wrist, WristConstants.wristUnderElevatorPt2Pos, 4, 1),
             new MoveArmToPos(s_Arm, ArmConstants.armUnderElevatorPos, 4.5)
        ),
        Commands.parallel(
            new MoveElevatorToPos(s_Elevator, ElevatorConstants.elevatorDownPos),
            Commands.parallel(
                new MoveArmToPos(s_Arm, ArmConstants.armDrivePos, 3.8),
                Commands.sequence(
                    Commands.parallel(
                    new MoveWristToPos(s_Wrist, WristConstants.wristReadyToFlipPos, 3.5, 1),
                    new MoveFlipperToPos(s_Flipper, FlipperConstants.flipperDrivePos)
                    ),
                    new MoveWristToPos(s_Wrist, WristConstants.wristDrivePos, 2.4, 0.9)
                )
            )
        )
    )
    );

    public static Command getAutoCommand(WhichAuto auto) {
        switch (auto) {
                case testAuto:
                        return chosenAuto = testAuto();
                case charge:
                        return chosenAuto = preloadChargeCenterAuto();
                case preloadMobilityCableCube:
                        return chosenAuto = preloadMobilityCableCube();
                case preloadMobilityOpenCube:
                        return chosenAuto = preloadMobilityOpenCube();
                case preloadMobilityCableCone:
                        return chosenAuto = preloadMobilityCableCone();
                case preloadMobilityOpenCone:  
                        return chosenAuto = preloadMobilityOpenCone();
                default:
                        break;
        }
        return chosenAuto = preloadCube();
}

    public enum WhichAuto {
        testAuto,
        charge,
        preload,
        preloadMobilityCableCube,
        preloadMobilityOpenCube,
        preloadMobilityCableCone,
        preloadMobilityOpenCone
}

public Command getChosenAuto() {
    return chosenAuto;
}

    public static void cancelLastCommand() {
        lastCommand.cancel();
    }

   /* public static Command followTrajectoryCommand(PathPlannerTrajectory traj) {

        lastCommand = new SequentialCommandGroup(
             new PPSwerveControllerCommand(
                 traj, 
                 s_Swerve::getPose, // Pose supplier
                 Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
                 new PIDController(5, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 new PIDController(5, 0, 0), // Y controller (usually the same values as X controller)
                 new PIDController(2, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 s_Swerve::setModuleStates, // Module states consumer
                 true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                 s_Swerve));
        return lastCommand;
     }*/

    static SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      s_Swerve::getPose,
      s_Swerve::resetOdometryAndHeading,
      Constants.Swerve.swerveKinematics,
      new PIDConstants(5, 0, 0),
      new PIDConstants(2, 0, 0),
      s_Swerve::setModuleStates,
      eventMap,
      true,
      s_Swerve
   );
   

     public static void driveForward(double distance, double speed) {
      double targetPose = s_Swerve.getPose().getX() + distance;
      while (s_Swerve.getPose().getX() < targetPose) {
         SmartDashboard.putNumber("Robot X", s_Swerve.getPose().getX());
            s_Swerve.drive(
                            new Translation2d(speed, 0).times(Constants.Swerve.maxSpeed),
                            0,
                            true,
                            true);
      }
      s_Swerve.drive(new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 0, true, true);
}


private static Command makeAuto(String path, double speed, double acceleration) {
   return autoBuilder.fullAuto(
       PathPlanner.loadPathGroup(path, speed, acceleration)
   );
}
     public static Command testAuto() {
        /*PathPlannerTrajectory traj = PathPlanner.loadPath("test", new PathConstraints(2.5, 2));
        return followTrajectoryCommand(traj);*/
        //return new InstantCommand(() -> driveForward(0.4, 0.2));
        return makeAuto("test", 1, 1);
     }

      public static Command preloadChargeCenterAuto() {
         return Commands.sequence(
         preloadCube(),
         makeAuto("1 + charge center pt1", 2.65, 2.5),
         makeAuto("1 + charge center pt2", 1.4, 2),
         makeAuto("1 + charge center pt3", 2.5, 2.2)
         );
     }

     public static Command preloadMobilityCableCube() {
      return Commands.sequence(
         preloadCube(),
         makeAuto("1 + mobility cable cube", 1, 1)
         );
     }

     public static Command preloadMobilityOpenCube() {
      return Commands.sequence(
         preloadCube(),
         makeAuto("1 + mobility open cube", 1, 1)
         );
     }

    public static Command preloadMobilityOpenCone() {
        return Commands.sequence(
            preloadCone()
           // makeAuto("1 + mobility open cone", 2, 1.5)
        );
    }

    public static Command preloadMobilityCableCone() {
        return Commands.sequence(
         preloadCone()/* 
         //makeAuto("1 + mobility cable cone", 2, 1.5)*/);
    }

     public static Command preloadCube() {
      return scorePreloadCube;
     }
     public static Command preloadCone() {
        return scorePreloadCone;
     }
   }

