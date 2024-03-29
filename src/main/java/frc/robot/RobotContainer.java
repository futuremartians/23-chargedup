package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.LED;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FlipperConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LED;
import frc.robot.Constants.PWMPorts;

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

    private boolean intakeCone;
    private boolean coneMode = false;
    
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
    //private final Limelight s_Limelight;
    private final Camera s_Camera;
    public static final LED m_led = new LED(PWMPorts.kBlinkin);
    



    

    /*Command Groups*/
    
    private final Command goToHighNodeScoringPosPt1 =
    Commands.sequence(
        Commands.sequence(
            new MoveArmToPos(s_Arm, ArmConstants.armDrivePos, 1.5),
            new MoveWristToPos(s_Wrist, WristConstants.wristReadyToFlipPos, 2, 1.2),
            new MoveFlipperToPos(s_Flipper, FlipperConstants.flipperScoringPos)
           // new MoveWristToPos(s_Wrist, WristConstants.wristAfterFlipPos, 2, 1.2)
        ),
    Commands.parallel(
        new MoveElevatorToPos(s_Elevator, ElevatorConstants.elevatorCommunityPos, 4.5),
        new MoveWristToPos(s_Wrist, -60000, 3.5, 1)
         )
    );

    private final Command goToHighNodeScoringPosPt2 = 
    Commands.parallel(
        new MoveElevatorToPos(s_Elevator, ElevatorConstants.elevatorUpPos, 5),
        Commands.sequence(
            Commands.waitSeconds(0.4),
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
    );
    
    private final Command goToHighNodeScoringPos = 
    Commands.sequence(
        Commands.sequence(
            new MoveArmToPos(s_Arm, ArmConstants.armDrivePos, 1.5),
            new MoveWristToPos(s_Wrist, WristConstants.wristReadyToFlipPos, 2, 1.2),
            new MoveFlipperToPos(s_Flipper, FlipperConstants.flipperScoringPos)
           // new MoveWristToPos(s_Wrist, WristConstants.wristAfterFlipPos, 2, 1.2)
        ),
    Commands.parallel(
        new MoveElevatorToPos(s_Elevator, ElevatorConstants.elevatorUpPos, 6),
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
                            ))
                    )
                    )
                )
            )
    );
    

    public final Command goToDriverPosFromTop = 
    Commands.sequence(
        Commands.parallel(
            new MoveWristToPos(s_Wrist, WristConstants.wristUnderElevatorPt2Pos, 4, 1),
             new MoveArmToPos(s_Arm, ArmConstants.armUnderElevatorPos, 4.5)
        ),
        Commands.parallel(
            new MoveElevatorToPos(s_Elevator, ElevatorConstants.elevatorDownPos, 1),
            Commands.parallel(
                new MoveArmToPos(s_Arm, ArmConstants.armDrivePos, 3.8),
                Commands.sequence(
                    Commands.parallel(
                    new MoveWristToPos(s_Wrist, WristConstants.wristReadyToFlipPos, 3.5, 1),
                    new MoveFlipperToPos(s_Flipper, FlipperConstants.flipperDrivePos)
                    ),
                    new MoveWristToPos(s_Wrist, WristConstants.wristDrivePos, 1.5, 0.9)
                )
            )
        )
    );

    public final Command goToDriverPosFromBottom = 
    Commands.sequence(
    Commands.parallel(
        new MoveArmToPos(s_Arm, 0, 2.65),
        new MoveElevatorToPos(s_Elevator, ElevatorConstants.elevatorDownPos, 1), 
        new MoveFlipperToPos(s_Flipper, 0),
        Commands.sequence(
            Commands.waitSeconds(0.15),
            new MoveWristToPos(s_Wrist, WristConstants.wristDrivePosWhileComingUp, 2.5, 1)
        )
    ),
    new MoveWristToPos(s_Wrist, WristConstants.wristDrivePos, 1, 1)
    );

    public final Command goToBottomIntakePos = 
    Commands.sequence(
     Commands.parallel(
            new MoveArmToPos(s_Arm, ArmConstants.armGroundIntakePos, 2),
            new MoveFlipperToPos(s_Flipper, FlipperConstants.flipperDrivePos),
            Commands.sequence(
                Commands.waitSeconds(0.3),
                new MoveWristToPos(s_Wrist, WristConstants.wristFoldedBeforeIntake, 1.6, 1)
            )
     ), 
      new MoveWristToPos(s_Wrist, WristConstants.wristIntakeBottomPos, 2, 1)
    );

    private final Command goToMediumNodeScoringPos = 
    /*Commands.sequence(
        Commands.sequence(
            new MoveArmToPos(s_Arm, ArmConstants.armDrivePos, 1.5),
            new MoveWristToPos(s_Wrist, WristConstants.wristReadyToFlipPos, 2, 1.2),
            new MoveFlipperToPos(s_Flipper, FlipperConstants.flipperScoringPos),
            new MoveWristToPos(s_Wrist, WristConstants.wristAfterFlipPos, 2, 1.2)
        ),
    Commands.parallel(
        new MoveElevatorToPos(s_Elevator, ElevatorConstants.elevatorUpPos),
        Commands.sequence(
            Commands.waitSeconds(0.2), 
            new MoveWristToPos(s_Wrist, WristConstants.wristUnderElevatorPt1Pos, 2.25, 1),
            Commands.parallel(
                new MoveWristToPos(s_Wrist, WristConstants.wristUnderElevatorPt2Pos, 3.7, 1),
                new MoveArmToPos(s_Arm, ArmConstants.armUnderElevatorPos, 2.4),
                Commands.sequence(
                    Commands.waitSeconds(0.4),
                    Commands.parallel(
                        new MoveArmToPos(s_Arm, ArmConstants.armMediumNodeScoringPos, 3.2),
                        Commands.sequence(
                            Commands.waitSeconds(0.9),
                            new MoveWristToPos(s_Wrist, WristConstants.wristMediumNodeScoringPos, 1.9, 0.9)
                            ))
                    )
                    )
                )
            )
    )*/
    Commands.sequence(
        new MoveWristToPos(s_Wrist, WristConstants.wristMediumNodeScoringIntakeSidePos, 2.4, 1),
        new MoveArmToPos(s_Arm, ArmConstants.armMediumNodeScoringIntakeSidePos, 1.7)
    );
    
    
   /* Commands.sequence(
        new MoveWristToPos(s_Wrist, WristConstants.wristReadyToFlipPos, 2, 1),
    Commands.parallel(
        new MoveArmToPos(s_Arm, ArmConstants.armMediumNodeScoringPos, 2),
        Commands.sequence(
        new MoveWristToPos(s_Wrist, WristConstants.wristSafeMediumNodePos, 1.4, 1),
        new MoveWristToPos(s_Wrist, WristConstants.wristMediumNodeScoringPos, 2, 1)
        )
    )*/
    

    public final Command goToDoubleSubstationConeIntakePos = 
    Commands.parallel(
        new MoveArmToPos(s_Arm, ArmConstants.armDoubleSubstationConIntakePos, 2.5),
        Commands.sequence(
            Commands.waitSeconds(0.3),
            new MoveWristToPos(s_Wrist, WristConstants.wristDoubleSubstationConeIntakePos, 2, 0.8)
        )
    );

    public final Command goToDoubleSubstationCubeIntakePos = 
    Commands.parallel(
        new MoveArmToPos(s_Arm, ArmConstants.armDoubleSubstationCubeIntakePos, 2.5),
        Commands.sequence(
            Commands.waitSeconds(0.2),
            new MoveWristToPos(s_Wrist, WristConstants.wristDoubleSubstationCubeIntakePos, 2, 0.8)
        )
    );

    public final Command goToSubstationCubeIntakePos = 
    Commands.parallel(
        new MoveArmToPos(s_Arm, ArmConstants.armGroundIntakePos, 2),
        Commands.sequence(
            Commands.waitSeconds(0.3),
            new MoveWristToPos(s_Wrist, WristConstants.wristSubstationCubeIntakePos, 1.8, 0.8)
        )
    );

    public final Command goToSubstationConeIntakePos = 
    Commands.parallel(
        new MoveArmToPos(s_Arm, ArmConstants.armSubstationConeIntakePos, 2),
        Commands.sequence(
            Commands.waitSeconds(0.3),
            new MoveWristToPos(s_Wrist, WristConstants.wristSubstationConeIntakePos, 1.8, 0.8)
        )
    );

    private Command singleSub;
        /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

       //s_Limelight = Limelight.getInstance();
       s_Camera = Camera.getInstance();

       
    
                                                                                                                                                                                                                          
        s_Swerve.resetOdometryAndHeading(new Pose2d());
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
          s_Elevator.setDefaultCommand(
            new ElevatorJoystick(
                s_Elevator, 
                () -> -operator.getRightX() * 0.4
            )
        );

        s_Arm.setDefaultCommand(
            new ArmJoystick(
                s_Arm, 
                () -> -operator.getRightY() * 0.4
            )
        );

         s_Wrist.setDefaultCommand(
            new WristJoystick(
                s_Wrist, 
                () -> -operator.getLeftY() * 0.4
            )
        );

        s_Flipper.setDefaultCommand(
            new FlipperJoystick(
                s_Flipper, 
                () -> -operator.getLeftX() * 0.2
            )
        );

        s_Intake.setDefaultCommand(
            new stallIntake(s_Intake,
             () -> intakeCone
            )
        );

        // Configure the button bindings
        configureButtonBindings();

        //s_Limelight.setIPDetails();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        driver.povUp().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        /* Operator Buttons */
        //Commands.parallel(new MoveWristToPos(s_Wrist, WristConstants.wristReadyToFlipPos), 
        //new MoveArmToPos(s_Arm, ArmConstants.armReadyToFlipPos))

        operator.povDown().onTrue(goToDriverPosFromTop);
        //operator.x().onTrue(new InstantCommand(() -> m_led.orange(), m_led));
       /* driver.rightTrigger().onTrue( Commands.sequence(
            new MoveArmToPos(s_Arm, ArmConstants.armDrivePos, 1.5),
            new MoveWristToPos(s_Wrist, WristConstants.wristReadyToFlipPos, 2, 1.2),
            new MoveFlipperToPos(s_Flipper, FlipperConstants.flipperScoringPos),
            new MoveWristToPos(s_Wrist, WristConstants.wristAfterFlipPos, 2, 1.2)
        ));*/
        operator.rightTrigger().whileTrue(new spinIntake(s_Intake, () -> -0.9));
        operator.leftTrigger().whileTrue(new spinIntake(s_Intake, () -> 1));
        operator.povUp().onTrue(goToHighNodeScoringPos);
        operator.rightTrigger().onTrue(new InstantCommand (() -> intakeCone = false));
        operator.leftTrigger().onTrue(new InstantCommand(() -> intakeCone = true));
        driver.leftBumper().onTrue(goToDriverPosFromBottom);
        driver.rightBumper().onTrue(goToBottomIntakePos);
        driver.a().onTrue(goToSubstationCubeIntakePos);  
        //driver.b().onTrue(goToSubstationConeIntakePos);
        //driver.y().onTrue(goToDoubleSubstationCubeIntakePos);
        operator.a().onTrue(Commands.parallel(
            new MoveArmToPos(s_Arm, ArmConstants.armDrivePos, 1),
            new MoveWristToPos(s_Wrist, WristConstants.wristAutoPreloadPos, 1.75, 0.8)
        ));


        //operator.y().onTrue(new InstantCommand(() -> singleSub = goToSubstationCubeIntakePos));
        //operator.povRight().onTrue(goToMediumNodeScoringPos);
       // driver.x().onTrue(goToDoubleSubstationConeIntakePos);
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    }

    private void singleSubIntake() {
        
        if (coneMode) {
            singleSub = goToSubstationConeIntakePos;
        } else {
            singleSub = goToSubstationCubeIntakePos;
        }
    }
}