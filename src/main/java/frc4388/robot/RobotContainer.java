/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

// Drive Systems
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc4388.utility.controller.XboxController;
import frc4388.utility.controller.DeadbandedXboxController;
import frc4388.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

// Commands
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

// Autos
import frc4388.robot.commands.Autos.PlaybackChooser;
import frc4388.robot.commands.Swerve.JoystickPlayback;
import frc4388.robot.commands.Swerve.JoystickRecorder;
import frc4388.utility.controller.VirtualController;
import frc4388.robot.commands.Swerve.neoJoystickPlayback;
import frc4388.robot.commands.Swerve.neoJoystickRecorder;
// import frc4388.robot.commands.Intake.ArmIntakeIn;
//import frc4388.robot.commands.Autos.AutoAlign;

// Subsystems
// import frc4388.robot.subsystems.LED;
// import frc4388.robot.subsystems.Limelight;
import frc4388.robot.subsystems.SwerveDrive;
// import frc4388.robot.subsystems.Shooter;
// import frc4388.robot.subsystems.Climber;
// import frc4388.robot.subsystems.Intake;

// Utilites
import frc4388.utility.DeferredBlock;
import frc4388.utility.configurable.ConfigurableString;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* RobotMap */
    public final RobotMap m_robotMap = new RobotMap();
    
    /* Subsystems */
    // private final LED m_robotLED = new LED();

    // private final Intake m_robotIntake = new Intake(m_robotMap.intakeMotor, m_robotMap.pivotMotor);

    public final SwerveDrive m_robotSwerveDrive = new SwerveDrive(m_robotMap.leftFront,
                                                                  m_robotMap.rightFront,
                                                                  m_robotMap.leftBack,
                                                                  m_robotMap.rightBack,
                                              
                                                                  m_robotMap.gyro);

    /* Controllers */
    private final DeadbandedXboxController m_driverXbox   = new DeadbandedXboxController(OIConstants.XBOX_DRIVER_ID);
    private final DeadbandedXboxController m_operatorXbox = new DeadbandedXboxController(OIConstants.XBOX_OPERATOR_ID);    
    // private final DeadbandedXboxController m_autoRecorderXbox = new DeadbandedXboxController(2);
    

    /* Virtual Controllers */
    // private final VirtualController m_virtualDriver = new VirtualController(0);
    // private final VirtualController m_virtualOperator = new VirtualController(1);

    // private Command intakeToShootStuff = new ArmIntakeIn(m_robotIntake, m_robotShooter);
    // private Command interrupt = new InstantCommand(() -> {}, m_robotIntake, m_robotShooter);

    // private SequentialCommandGroup intakeToShoot = new SequentialCommandGroup(
    //     new InstantCommand(() -> m_robotIntake.PIDIn()),
    //     new InstantCommand(() -> m_robotShooter.idle())
    //     // new InstantCommand(() -> m_driverXbox.setRumble(RumbleType.kRightRumble, 1.0)).andThen(new WaitCommand(0.2)).andThen(new InstantCommand(() -> m_driverXbox.setRumble(RumbleType.kRightRumble, 0.0))),
    //     // new InstantCommand(() -> m_robotShooter.spin())
    // );


    // ! Teleop Commands

    //private AutoAlign autoAlign = new AutoAlign(m_robotSwerveDrive, limelight);

    // private SequentialCommandGroup autoShoot = new SequentialCommandGroup(
    //     // MoveToSpeaker,
    //     //autoAlign,
    //     new InstantCommand(() -> m_robotShooter.spin()),
    //     new WaitCommand(3.0),
    //     new InstantCommand(() -> m_robotIntake.handoff(), m_robotIntake),
    //     new WaitCommand(3.0),
    //     new InstantCommand(() -> m_robotShooter.idle())
    //     // new InstantCommand(() -> autoAlign.reverse()),
    //    // autoAlign.asProxy()
    // );


    

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureButtonBindings();        
        // configureVirtualButtonBindings();
        // new Trigger(() -> autoplaybackName.get().equals(lastAutoName)).onTrue(new InstantCommand(() -> changeAuto()));
        // new DeferredBlock(() -> m_robotSwerveDrive.resetGyro());


        //     DriverStation.silenceJoystickConnectionWarning(true);
        //     // CameraServer.startAutomaticCapture();

        //     /* Default Commands */
        //     // drives the robot with a two-axis input from the driver controller
            // ! Swerve Drive Default Command (Regular Rotation)

        // m_robotSwerveDrive.setDefaultCommand(new RunCommand(() -> {
        //     m_robotMap.rightFront.go(getDeadbandedDriverController().getLeft());
        // }
        // m_robotSwerveDrive.setDefaultCommand(new RunCommand(() -> {
        //     m_robotSwerveDrive.driveWithInput(getDeadbandedDriverController().getLeft(),
        //                                     getDeadbandedDriverController().getRight(),
        //                         true);
        // }, m_robotSwerveDrive)
        // .withName("SwerveDrive DefaultCommand"));
        // m_robotSwerveDrive.setToSlow();

        // ! Swerve Drive Default Command (Orientation Rotation)
        // m_robotSwerveDrive.setDefaultCommand(new RunCommand(() -> {
        //     m_robotSwerveDrive.driveWithInputOrientation(getDeadbandedDriverController().getLeft(), 
        //                                                  getDeadbandedDriverController().getRightX(), 
        //                                                  getDeadbandedDriverController().getRightY(), 
        //                                                  true);
        // }, m_robotSwerveDrive));

        m_robotSwerveDrive.setDefaultCommand(new RunCommand(() -> {
            m_robotSwerveDrive.driveWithInput(
                                            getDeadbandedDriverController().getLeft(), 
                                            getDeadbandedDriverController().getRight(),
                                            true);
        }, m_robotSwerveDrive));



        // .withName("SwerveDrive OrientationCommand"));
        // continually sends updates to the Blinkin LED controller to keep the lights on
        // m_robotLED.setDefaultCommand(new RunCommand(() -> m_robotLED.updateLED(), m_robotLED));

    }

    // private void changeAuto() {
    //     autoPlayback.unloadAuto();
    //     autoPlayback.loadAuto();
    //     lastAutoName = autoplaybackName.get();
    //     System.out.println("AUTO: Changed auto to; `" + lastAutoName + "`");
    // }
    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        //no auto
        // return autoPlayback;
        //return playbackChooser.getCommand();
        return new Command() {};
    }

    /**
     * A button binding for two controllers, preferably an {@link DeadbandedXboxController Xbox Controller} and {@link VirtualController Virtual Xbox Controller}
     * @param joystickA A controller
     * @param joystickB A controller
     * @param buttonNumber The button to bind to
     */
    public Trigger DualJoystickButton(GenericHID joystickA, GenericHID joystickB, int buttonNumber) {
        return new Trigger(() -> (joystickA.getRawButton(buttonNumber) || joystickB.getRawButton(buttonNumber)));
    }

    /**
     * Add your docs here.
     */
    public DeadbandedXboxController getDeadbandedDriverController() {
        return this.m_driverXbox;
    }

    public DeadbandedXboxController getDeadbandedOperatorController() {
        return this.m_operatorXbox;
    }

    // public VirtualController getVirtualDriverController() {
    //     return m_virtualDriver;
    // }

    // public VirtualController getVirtualOperatorController() {
    //     return m_virtualOperator;
    // }
}
