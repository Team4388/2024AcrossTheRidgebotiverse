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
import frc4388.utility.controller.XboxController;
import frc4388.utility.controller.DeadbandedXboxController;
import frc4388.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// Commands
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// Autos
import frc4388.robot.commands.Intake.ArmIntakeIn;
import frc4388.utility.controller.VirtualController;
import frc4388.robot.commands.Swerve.neoJoystickPlayback;
import frc4388.robot.commands.Swerve.neoJoystickRecorder;

// Subsystems
import frc4388.robot.subsystems.Climber;
import frc4388.robot.subsystems.Intake;
import frc4388.robot.subsystems.Limelight;
import frc4388.robot.subsystems.Shooter;
// import frc4388.robot.subsystems.LED;
import frc4388.robot.subsystems.SwerveDrive;

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

    private final Intake m_robotIntake = new Intake(m_robotMap.intakeMotor, m_robotMap.pivotMotor);

    public final SwerveDrive m_robotSwerveDrive = new SwerveDrive(m_robotMap.leftFront,
                                                                  m_robotMap.rightFront,
                                                                  m_robotMap.leftBack,
                                                                  m_robotMap.rightBack,
                                              
                                                                  m_robotMap.gyro);

    /* Controllers */
    private final DeadbandedXboxController m_driverXbox   = new DeadbandedXboxController(OIConstants.XBOX_DRIVER_ID);
    private final DeadbandedXboxController m_operatorXbox = new DeadbandedXboxController(OIConstants.XBOX_OPERATOR_ID);    
    private final DeadbandedXboxController m_autoRecorderXbox = new DeadbandedXboxController(OIConstants.XBOX_PROGRAMMER_ID);
    
    private final Limelight limelight = new Limelight();

    private final Shooter m_robotShooter = new Shooter(m_robotMap.leftShooter, m_robotMap.rightShooter, limelight);

    private final Climber m_robotClimber = new Climber(m_robotMap.climbMotor);

    /* Virtual Controllers */
    private final VirtualController m_virtualDriver = new VirtualController(0);
    private final VirtualController m_virtualOperator = new VirtualController(1);

    private Command intakeToShootStuff = new ArmIntakeIn(m_robotIntake, m_robotShooter);
    private Command interrupt = new InstantCommand(() -> {}, m_robotIntake, m_robotShooter);

    // ! Teleop Commands

    private SequentialCommandGroup intakeToShoot = new SequentialCommandGroup(
        new InstantCommand(() -> m_robotIntake.PIDIn()),
        new InstantCommand(() -> m_robotShooter.idle())
    );
    
    private SequentialCommandGroup intakeNotePullInIdle = new SequentialCommandGroup(
        intakeToShootStuff, intakeToShoot, 
        new InstantCommand(() -> m_robotShooter.idle())
    );

    private SequentialCommandGroup turnOffShoot = new SequentialCommandGroup(
        new InstantCommand(() -> m_robotShooter.stop(), m_robotShooter)
        // new InstantCommand(() -> m_robotIntake.talonStopIntakeMotors(), m_robotIntake)
    );

    private SequentialCommandGroup emergencyRetract = new SequentialCommandGroup(
        interrupt,
        new InstantCommand(() -> m_robotIntake.PIDIn(), m_robotIntake),
        new InstantCommand(() -> m_robotIntake.stopIntakeMotors(), m_robotIntake)
    );

    // ! /*  Autos */
    private String lastAutoName = "four_note_taxi_kracken.auto";
    private ConfigurableString autoplaybackName = new ConfigurableString("Auto Playback Name", lastAutoName);
    private neoJoystickPlayback autoPlayback = new neoJoystickPlayback(m_robotSwerveDrive, 
    () -> autoplaybackName.get(), // lastAutoName
           new VirtualController[]{getVirtualDriverController(), getVirtualOperatorController()},
           true, false);
    
    private neoJoystickPlayback ampShoot = new neoJoystickPlayback(m_robotSwerveDrive, "Amp_shoot.auto",
        new VirtualController[]{getVirtualDriverController(), getVirtualOperatorController()},
        false, true);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureButtonBindings();        
        configureVirtualButtonBindings();
        new DeferredBlock(() -> m_robotSwerveDrive.resetGyroFlip());
        DriverStation.silenceJoystickConnectionWarning(true);
        // CameraServer.startAutomaticCapture();

        /* Default Commands */
        // ! Swerve Drive Default Command (Regular Rotation)
        // drives the robot with a two-axis input from the driver controller
        m_robotSwerveDrive.setDefaultCommand(new RunCommand(() -> {
            m_robotSwerveDrive.driveWithInput(getDeadbandedDriverController().getLeft(),
                                            getDeadbandedDriverController().getRight(),
                                true);
        }, m_robotSwerveDrive)
        .withName("SwerveDrive DefaultCommand"));
        m_robotSwerveDrive.setToSlow();

        // ! Swerve Drive One Module Test
        // m_robotSwerveDrive.setDefaultCommand(new RunCommand(() -> {
        //     m_robotMap.rightFront.go(getDeadbandedDriverController().getLeft());
        // }

        // ! Swerve Drive Default Command (Orientation Rotation)
        // m_robotSwerveDrive.setDefaultCommand(new RunCommand(() -> {
        //     m_robotSwerveDrive.driveWithInputOrientation(getDeadbandedDriverController().getLeft(), 
        //                                                  getDeadbandedDriverController().getRightX(), 
        //                                                  getDeadbandedDriverController().getRightY(), 
        //                                                  true);
        // }, m_robotSwerveDrive))
        // .withName("SwerveDrive OrientationCommand"));
        // continually sends updates to the Blinkin LED controller to keep the lights on
        // m_robotLED.setDefaultCommand(new RunCommand(() -> m_robotLED.updateLED(), m_robotLED));

        // m_robotSwerveDrive.setDefaultCommand(new RunCommand(() -> {
        //     m_robotSwerveDrive.driveWithInput(
        //                                     getDeadbandedDriverController().getLeft(), 
        //                                     getDeadbandedDriverController().getRight(),
        //                                     true);
        // }, m_robotSwerveDrive));




    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        // ? /* Driver Buttons */

        DualJoystickButton(getDeadbandedDriverController(), getVirtualDriverController(), XboxController.A_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.resetGyroFlip()));
            
        // ! /* Speed */
        new JoystickButton(getDeadbandedDriverController(), XboxController.RIGHT_BUMPER_BUTTON) // final
            .onTrue(new InstantCommand(()  -> m_robotSwerveDrive.shiftUp()));
        
        new JoystickButton(getDeadbandedDriverController(), XboxController.LEFT_BUMPER_BUTTON) // final
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.shiftDown()));

        new Trigger(() -> getDeadbandedDriverController().getPOV() == 270)
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.shiftDownRot()));

        new Trigger(() -> getDeadbandedDriverController().getPOV() == 90)
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.shiftUpRot()));
          
        // ?  /* Operator Buttons */
            
        DualJoystickButton(getDeadbandedOperatorController(), getVirtualOperatorController(), XboxController.Y_BUTTON)
        .onTrue(new InstantCommand(() -> m_robotIntake.PIDIn()))
            .onFalse(new InstantCommand(() -> m_robotIntake.stopArmMotor()));
            
        DualJoystickButton(getDeadbandedOperatorController(), getVirtualOperatorController(), XboxController.X_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotIntake.PIDOut()))
            .onFalse(new InstantCommand(() -> m_robotIntake.stopArmMotor()));

        DualJoystickButton(getDeadbandedOperatorController(), getVirtualOperatorController(), XboxController.A_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotIntake.handoff()))
            .onFalse(new InstantCommand(() -> m_robotIntake.stopIntakeMotors()));
            
        DualJoystickButton(getDeadbandedOperatorController(), getVirtualOperatorController(), XboxController.B_BUTTON)
             .onTrue(emergencyRetract);


        // Override Intake Position encoder: out
        new JoystickButton(getDeadbandedOperatorController(), XboxController.BACK_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotIntake.setPivotEncoderPosition(-55), m_robotIntake));
            
        // Override Intake Position encoder: in
        new JoystickButton(getDeadbandedOperatorController(), XboxController.START_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotIntake.setPivotEncoderPosition(-6.2), m_robotIntake));

        DualJoystickButton(getDeadbandedOperatorController(), getVirtualOperatorController(), XboxController.LEFT_BUMPER_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotShooter.spin(0.5), m_robotShooter))
            .onFalse(turnOffShoot);
            

        DualJoystickButton(getDeadbandedOperatorController(), getVirtualOperatorController(), XboxController.RIGHT_BUMPER_BUTTON)
            .onTrue(intakeNotePullInIdle)
            .onFalse(new InstantCommand(() -> m_robotIntake.PIDIn()));
            
        // Spins up shooter, no wind down
        DualJoystickButton(getDeadbandedOperatorController(), getVirtualOperatorController(), XboxController.LEFT_JOYSTICK_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotShooter.spin(), m_robotShooter));

        DualJoystickButton(getDeadbandedOperatorController(), getVirtualOperatorController(), XboxController.RIGHT_JOYSTICK_BUTTON)
            .onTrue(emergencyRetract);

        new Trigger(() -> getDeadbandedOperatorController().getRawAxis(XboxController.RIGHT_TRIGGER_AXIS) > 0.5)
            .onTrue(new InstantCommand(() -> m_robotClimber.climbOut()))
            .onFalse(new InstantCommand(() -> m_robotClimber.stopClimb()));

        new Trigger(() -> getDeadbandedOperatorController().getRawAxis(XboxController.LEFT_TRIGGER_AXIS) > 0.5)
            .onTrue(new InstantCommand(() -> m_robotClimber.climbIn()))
            .onFalse(new InstantCommand(() -> m_robotClimber.stopClimb()));

        // new Trigger(() -> getDeadbandedOperatorController().getPOV() == 0)
        //     .onTrue(new InstantCommand(() -> m_robotIntake.ampOuttake(0.5)));
        
        new Trigger(() -> getDeadbandedOperatorController().getPOV() != -1)
            .onTrue(ampShoot)
            .onFalse(new InstantCommand(() -> m_robotShooter.stop(), m_robotShooter, m_robotSwerveDrive));
        
        // ? /* Programer Buttons (Controller 3)*/

        // * /* Auto Recording */
        new JoystickButton(m_autoRecorderXbox, XboxController.LEFT_BUMPER_BUTTON)
            .whileTrue(new neoJoystickRecorder(m_robotSwerveDrive,
                        new DeadbandedXboxController[]{getDeadbandedDriverController(), getDeadbandedOperatorController()},
                                            () -> autoplaybackName.get()))
            .onFalse(new InstantCommand());
        
        new JoystickButton(m_autoRecorderXbox, XboxController.RIGHT_BUMPER_BUTTON)
            .onTrue(new neoJoystickPlayback(m_robotSwerveDrive,
            () -> autoplaybackName.get(),
            new VirtualController[]{getVirtualDriverController(), getVirtualOperatorController()},
            true, false))
            .onFalse(new InstantCommand());
    }
    
    /**
     * This method is used to replcate {@link Trigger Triggers} for {@link VirtualController Virtual Controllers}. <p/>
     * Please use {@link RobotContainer#DualJoystickButton} in {@link RobotContainer#configureButtonBindings} for standard buttons.
     */
    private void configureVirtualButtonBindings() {

        // ? /* Driver Buttons */
        
        /* Notice: the following buttons have not been replicated
         * Swerve Drive Slow and Fast mode Gear Shifts : Fast mode is known to cause drift, so we disable that feature in Autoplayback
         * Swerve Drive Rotation Gear Shifts           : Same reason as Slow and Fast mode.
         * Auto Recording controls                     : We don't want an Null Ouroboros for an auto.
         */

        // ? /* Operator Buttons */

        /* Notice: the following buttons have not been replicated
         * Override Intake Position Encoder : It's an emergancy overide, for when the position of intake when the robot boots, the intake is not inside the robot.
         *                                    We don't need it in an auto.
         * Climbing controls                : We don't need to climb in auto.
         */
        
         // ? Notice: the Programer Buttons are not to be replicated because they are designed for debuging the robot, and do not need to be replicated in auto.

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        //no auto
        return autoPlayback;
        //return playbackChooser.getCommand();
        // return new Command() {};
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

    public DeadbandedXboxController getDeadbandedDriverController() {
        return this.m_driverXbox;
    }

    public DeadbandedXboxController getDeadbandedOperatorController() {
        return this.m_operatorXbox;
    }

    public VirtualController getVirtualDriverController() {
        return m_virtualDriver;
    }

    public VirtualController getVirtualOperatorController() {
        return m_virtualOperator;
    }
}
