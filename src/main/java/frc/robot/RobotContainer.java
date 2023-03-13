package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FlipperConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    /* Drive Controls 
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    private final int elevatorPowerAxis = XboxController.Axis.kRightY.value;*/

    /* Driver Buttons */
    /*private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton elevatorUp = new JoystickButton(operator, XboxController.Button.kY.value);*/
    

    /* Subsystems */
    private final Swerve s_Swerve = Swerve.getInstance();
    private final Elevator s_Elevator =  Elevator.getInstance();
    private final Arm s_Arm = Arm.getInstance();
    private final Wrist s_Wrist = Wrist.getInstance();
    private final Flipper s_Flipper = Flipper.getInstance();
    private final Intake s_Intake = Intake.getInstance();
    private final Limelight s_Limelight;

    /*Command Groups*/
    /*private final Command goToHighPoleScoringPos = Commands.sequence(Commands.parallel(new MoveElevatorToPos(s_Elevator, ElevatorConstants.elevatorUpPos), 
    Commands.sequence(Commands.parallel(new MoveWristToPos(s_Wrist, WristConstants.wristReadyToFlipPos), 
    new MoveArmToPos(s_Arm, ArmConstants.armReadyToFlipPos)), 
    Commands.parallel(new MoveFlipperToPos(s_Flipper, FlipperConstants.flipperScoringPos), 
    new MoveArmToPos(s_Arm, ArmConstants.armReadyForScoringPos), 
    new MoveWristToPos(s_Wrist, WristConstants.wristReadyForScoringPos)))), 
    Commands.sequence(Commands.parallel(new MoveArmToPos(s_Arm, ArmConstants.armScoringPos), 
    new MoveWristToPos(s_Wrist, WristConstants.wristUnderElevatorPos)), new MoveWristToPos(s_Wrist, WristConstants.wristScoringPos)));*/

    public final Command goToDriverPosFromBottom = 
    Commands.sequence(
    Commands.parallel(
        new MoveArmToPos(s_Arm, 0, 2),
        new MoveElevatorToPos(s_Elevator, 0), 
        new MoveFlipperToPos(s_Flipper, 0)
    )//,
    //new MoveWristToPos(s_Wrist, 0, 3, 1)
    );

    public final Command goToBottomIntakePos = 
    new ParallelCommandGroup( new MoveArmToPos(s_Arm, ArmConstants.intakeDownPos, 1.4), 
    new MoveWristToPos(s_Wrist, WristConstants.wristIntakeBottomPos, 3, 3.8)
    );


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        s_Limelight = Limelight.getInstance();

        s_Swerve.resetOdometry(new Pose2d());
        s_Swerve.zeroGyro();
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getLeftY(), 
                () -> -driver.getLeftX(), 
                () -> -driver.getRightX(), 
                () -> false
            )
        );
        /*  s_Elevator.setDefaultCommand(
            new ElevatorJoystick(
                s_Elevator, 
                () -> -operator.getRightY()*0.4
            )
        );*/

        s_Arm.setDefaultCommand(
            new ArmJoystick(
                s_Arm, 
                () -> -operator.getRightY()*0.4
            )
        );

       /*  s_Wrist.setDefaultCommand(
            new WristJoystick(
                s_Wrist, 
                () -> -operator.getLeftY()*0.4
            )
        );*/

        s_Flipper.setDefaultCommand(
            new FlipperJoystick(
                s_Flipper, 
                () -> -operator.getLeftX()*0.2
            )
        );

        s_Intake.setDefaultCommand(
            new spinIntake(s_Intake,
             () -> 0
            )
        );

        s_Wrist.setDefaultCommand(
            new HoldWristAtPos(s_Wrist,  
            1)
        );

        // Configure the button bindings
        configureButtonBindings();

        s_Limelight.setIPDetails();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        driver.y().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        /* Operator Buttons */
        operator.povUp().onTrue(goToDriverPosFromBottom);
        //Commands.parallel(new MoveWristToPos(s_Wrist, WristConstants.wristReadyToFlipPos), 
        //new MoveArmToPos(s_Arm, ArmConstants.armReadyToFlipPos))

        operator.povDown().onTrue(new MoveFlipperToPos(s_Flipper, FlipperConstants.flipperScoringPos));
        operator.rightTrigger().whileTrue(new spinIntake(s_Intake, () -> 0.5));
        operator.leftTrigger().whileTrue(new spinIntake(s_Intake, () -> -0.5));
        operator.y().onTrue(new MoveElevatorToPos(s_Elevator, ElevatorConstants.elevatorUpPos));
        operator.a().onTrue(new MoveElevatorToPos(s_Elevator, ElevatorConstants.elevatorDownPos));
        operator.rightBumper().onTrue(goToBottomIntakePos);
        operator.povRight().onTrue(new MoveWristToPos(s_Wrist, WristConstants.wristReadyToFlipPos, 2.5, 0.9));
        //operator.leftBumper().onTrue(new MoveArmToPos(s_Arm, ArmConstants.armDrivePos));

        //operator.b().onTrue(goToHighPoleScoringPos);

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    }
}
