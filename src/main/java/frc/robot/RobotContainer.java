package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ElevatorConstants;
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

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

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
       /*  s_elevator.setDefaultCommand(
            new ElevatorJoystick(
                s_elevator, 
                () -> -operator.getRightY()*0.4
            )
        );*/

        s_Arm.setDefaultCommand(
            new ArmJoystick(
                s_Arm, 
                () -> -operator.getRightY()*0.2
            )
        );

        // Configure the button bindings
        configureButtonBindings();
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
        operator.povUp().onTrue(new MoveElevatorToPos(s_Elevator, ElevatorConstants.elevatorUpPos));
        operator.povDown().onTrue(new MoveElevatorToPos(s_Elevator, ElevatorConstants.elevatorDownPos));
        
        //54240 upper limit -48500 lower limit


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    }
}
