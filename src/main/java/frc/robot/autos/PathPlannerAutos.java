package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import java.util.List;

import com.pathplanner.lib.PathConstraints;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
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

    private static Command lastCommand;
    private static Command chosenAuto;

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

    public static Command getAutoCommand(WhichAuto auto) {
        switch (auto) {
                case testAuto:
                        return chosenAuto = testAuto();
                case charge:
                        return chosenAuto = preloadChargeCenterAuto();
                case preload:
                        return chosenAuto = preloadCube();
                case preloadMobilityCable:
                        return chosenAuto = preloadMobilityCable();
                case preloadMobilityOpen:
                        return chosenAuto = preloadMobilityOpen();
        }
        return null;
}

    public enum WhichAuto {
        testAuto,
        charge,
        preload,
        preloadMobilityCable,
        preloadMobilityOpen
}

public Command getChosenAuto() {
    return chosenAuto;
}

    public static void cancelLastCommand() {
        lastCommand.cancel();
    }

    public static Command followTrajectoryCommand(PathPlannerTrajectory traj) {

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
     }

     public static Command testAuto() {
        PathPlannerTrajectory traj = PathPlanner.loadPath("test", new PathConstraints(2.5, 2));
        return followTrajectoryCommand(traj);
     }

     public static Command preloadChargeCenterAuto() {
        PathPlannerTrajectory traj = PathPlanner.loadPath("1 + charge", new PathConstraints(2.5, 2));
        return followTrajectoryCommand(traj);
     }

     public static Command preloadMobilityCable() {
        PathPlannerTrajectory traj = PathPlanner.loadPath("1 + mobility", new PathConstraints(2.5, 2));
        return followTrajectoryCommand(traj);
     }

     public static Command preloadMobilityOpen() {
        PathPlannerTrajectory traj = PathPlanner.loadPath("1 + mobility open", new PathConstraints(2.5, 2));
        return followTrajectoryCommand(traj);
     }

     public static Command preloadCube() {
      return scorePreloadCube;
     }
   }