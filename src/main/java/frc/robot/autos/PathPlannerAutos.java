package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import java.util.List;

import com.pathplanner.lib.PathConstraints;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class PathPlannerAutos {

    private static Swerve s_Swerve = new Swerve();
    private static Command lastCommand;

    public static void cancelLastCommand() {
        lastCommand.cancel();
    }

    public Command followTrajectoryCommand(PathPlannerTrajectory traj) {

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
}
