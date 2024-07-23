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
import frc4388.robot.commands.Intake.ArmIntakeIn;
//import frc4388.robot.commands.Autos.AutoAlign;

// Subsystems
import frc4388.robot.subsystems.LED;
import frc4388.robot.subsystems.Limelight;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.robot.subsystems.Shooter;
import frc4388.robot.subsystems.Climber;
import frc4388.robot.subsystems.Intake;

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
    private final RobotMap m_robotMap = new RobotMap();
    
    /* Subsystems */
    private final LED m_robotLED = new LED();

    private final Intake m_robotIntake = new Intake(m_robotMap.intakeMotor, m_robotMap.pivotMotor);

    public final SwerveDrive m_robotSwerveDrive = new SwerveDrive(m_robotMap.leftFront,
                                                                  m_robotMap.rightFront,
                                                                  m_robotMap.leftBack,
                                                                  m_robotMap.rightBack,
                                              
                                                                  m_robotMap.gyro);
    /* Limelight */
    private final Limelight limelight = new Limelight();

    private final Shooter m_robotShooter = new Shooter(m_robotMap.leftShooter, m_robotMap.rightShooter, limelight);

    private final Climber m_robotClimber = new Climber(m_robotMap.climbMotor);

    /* Controllers */
    private final DeadbandedXboxController m_driverXbox = new DeadbandedXboxController(OIConstants.XBOX_DRIVER_ID);
    private final DeadbandedXboxController m_operatorXbox = new DeadbandedXboxController(OIConstants.XBOX_OPERATOR_ID);    
    private final DeadbandedXboxController m_autoRecorderXbox = new DeadbandedXboxController(2);
    

    /* Virtual Controllers */
    private final VirtualController m_virtualDriver = new VirtualController(0);
    private final VirtualController m_virtualOperator = new VirtualController(1);

    private Command intakeToShootStuff = new ArmIntakeIn(m_robotIntake, m_robotShooter);
    private Command interrupt = new InstantCommand(() -> {}, m_robotIntake, m_robotShooter);

    private SequentialCommandGroup intakeToShoot = new SequentialCommandGroup(
        new InstantCommand(() -> m_robotIntake.PIDIn()),
        new InstantCommand(() -> m_robotShooter.idle())
        // new InstantCommand(() -> m_driverXbox.setRumble(RumbleType.kRightRumble, 1.0)).andThen(new WaitCommand(0.2)).andThen(new InstantCommand(() -> m_driverXbox.setRumble(RumbleType.kRightRumble, 0.0))),
        // new InstantCommand(() -> m_robotShooter.spin())
    );


    // ! Teleop Commands

    //private AutoAlign autoAlign = new AutoAlign(m_robotSwerveDrive, limelight);

    private SequentialCommandGroup autoShoot = new SequentialCommandGroup(
        // MoveToSpeaker,
        //autoAlign,
        new InstantCommand(() -> m_robotShooter.spin()),
        new WaitCommand(3.0),
        new InstantCommand(() -> m_robotIntake.handoff(), m_robotIntake),
        new WaitCommand(3.0),
        new InstantCommand(() -> m_robotShooter.idle())
        // new InstantCommand(() -> autoAlign.reverse()),
       // autoAlign.asProxy()
    );


    private SequentialCommandGroup i = new SequentialCommandGroup(
        intakeToShootStuff, intakeToShoot, 
        new InstantCommand(() -> m_robotShooter.idle())
    );

    private SequentialCommandGroup ejectToShoot = new SequentialCommandGroup(
        new InstantCommand(() -> m_robotShooter.spin(), m_robotShooter),
        new WaitCommand(0.75),
        new InstantCommand(() -> m_robotIntake.handoff(), m_robotIntake)
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

    private SequentialCommandGroup ampShoot = new SequentialCommandGroup(
        new InstantCommand(() -> m_robotIntake.ampPosition()),
        new InstantCommand(() -> m_robotIntake.ampOuttake(0.1)) //TODO: Find Actual Speed
    );

    // ! /*  Autos */
    // private Command taxi = new JoystickPlayback(m_robotSwerveDrive, "Taxi.txt");
    private String lastAutoName = "final_red_center_4note_taxi.auto";
    private ConfigurableString autoplaybackName = new ConfigurableString("Auto Playback Name", lastAutoName);
    private neoJoystickPlayback autoPlayback = new neoJoystickPlayback(m_robotSwerveDrive, 
    () -> autoplaybackName.get(), // lastAutoName
           new VirtualController[]{getVirtualDriverController(), getVirtualOperatorController()},
           true, false);

    // private PlaybackChooser playbackChooser = new PlaybackChooser(m_robotSwerveDrive)
    //         .addOption("Taxi Auto", taxi.asProxy())
    //         .addOption("One Note Auto Starting in Front of Speaker", oneNoteStartingSpeaker.asProxy())
    //         .addOption("Stay One Note Auto Starting in Front of Speaker", oneNoteStartingSpeakerStationary.asProxy())
    //         // .addOption("One Note Auto Starting from Left Position", oneNoteStartingFromLeft.asProxy())
    //         // .addOption("One Note Auto Starting from Right Position", oneNoteStartingFromRight.asProxy())
    //         .addOption("Two Note Starting in Front of Speaker", twoNoteStartingFromSpeaker.asProxy())
    //         .addOption("Stay Two Note Starting in Front of Speaker", stayTwoNoteStartingFromSpeaker.asProxy())
    //         .buildDisplay();    

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureButtonBindings();        
        configureVirtualButtonBindings();
        // new Trigger(() -> autoplaybackName.get().equals(lastAutoName)).onTrue(new InstantCommand(() -> changeAuto()));
        new DeferredBlock(() -> m_robotSwerveDrive.resetGyro());


            DriverStation.silenceJoystickConnectionWarning(true);
            // CameraServer.startAutomaticCapture();

            /* Default Commands */
            // drives the robot with a two-axis input from the driver controller
            // ! Swerve Drive Default Command (Regular Rotation)
            m_robotSwerveDrive.setDefaultCommand(new RunCommand(() -> {
                m_robotSwerveDrive.driveWithInput(getDeadbandedDriverController().getRightTrigger(),
                                                getDeadbandedDriverController().getRight().times(-1),
                                    false);
            }, m_robotSwerveDrive)
            .withName("SwerveDrive DefaultCommand"));
            m_robotSwerveDrive.hellsHorses();

        // ! Swerve Drive Default Command (Orientation Rotation)
        // m_robotSwerveDrive.setDefaultCommand(new RunCommand(() -> {
        //     m_robotSwerveDrive.driveWithInputOrientation(getDeadbandedDriverController().getLeft(), 
        //                                                  getDeadbandedDriverController().getRightX(), 
        //                                                  getDeadbandedDriverController().getRightY(), 
        //                                                  true);
        // }, m_robotSwerveDrive)
        // .withName("SwerveDrive OrientationCommand"));
        // continually sends updates to the Blinkin LED controller to keep the lights on
        m_robotLED.setDefaultCommand(new RunCommand(() -> m_robotLED.updateLED(), m_robotLED));

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
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.resetGyroFlip(), m_robotSwerveDrive));

        DualJoystickButton(getDeadbandedDriverController(), getVirtualDriverController(), XboxController.BACK_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.resetGyroRightBlue()))
            .onFalse(new InstantCommand(() -> m_robotSwerveDrive.add180()));

        DualJoystickButton(getDeadbandedDriverController(), getVirtualDriverController(), XboxController.START_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.resetGyroRightAmp()))
            .onFalse(new InstantCommand(() -> m_robotSwerveDrive.add180()));

       // *  /* D-Pad Stuff */
    //    new Trigger(() -> getDeadbandedDriverController().getPOV() == 0)
    //         .whileTrue(new InstantCommand(() -> m_robotSwerveDrive.driveWithInput(new Translation2d(0, 1),
    //                                                                            new Translation2d(0, 0),
    //                                                                            true), m_robotSwerveDrive))
    //         .onFalse(new InstantCommand(() -> m_robotSwerveDrive.driveWithInput(new Translation2d(0, 0), 
    //                                                                             new Translation2d(0, 0), 
    //                                                                             true)));

    //    new Trigger(() -> getDeadbandedDriverController().getRawAxis(XboxController.TOP_BOTTOM_DPAD_AXIS) > -0.9)
    //         .onTrue(new InstantCommand(() -> m_robotSwerveDrive.driveWithInput(new Translation2d(0, -1),
    //                                                                            new Translation2d(0, 0),
    //                                                                            true)))
    //         .onFalse(new InstantCommand(() -> m_robotSwerveDrive.driveWithInput(new Translation2d(0, 0), 
    //                                                                             new Translation2d(0, 0), 
    //                                                                             true)));

    //    new Trigger(() -> getDeadbandedDriverController().getRawAxis(XboxController.LEFT_RIGHT_DPAD_AXIS) > 0.9)
    //         .onTrue(new InstantCommand(() -> m_robotSwerveDrive.driveWithInput(new Translation2d(1, 0),
    //                                                                            new Translation2d(0, 0),
    //                                                                            true)))
    //         .onFalse(new InstantCommand(() -> m_robotSwerveDrive.driveWithInput(new Translation2d(0, 0), 
    //                                                                             new Translation2d(0, 0), 
    //                                                                             true)));

    //    new Trigger(() -> getDeadbandedDriverController().getRawAxis(XboxController.LEFT_RIGHT_DPAD_AXIS) > -0.9)
    //         .onTrue(new InstantCommand(() -> m_robotSwerveDrive.driveWithInput(new Translation2d(-1, 0),
    //                                                                            new Translation2d(0, 0),
    //                                                                            true)))
    //         .onFalse(new InstantCommand(() -> m_robotSwerveDrive.driveWithInput(new Translation2d(0, 0), 
    //                                                                             new Translation2d(0, 0), 
    //                                                                             true)));
        
        // ! /* Auto Recording */
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

        // ! /* Speed */
        new JoystickButton(getDeadbandedDriverController(), XboxController.RIGHT_BUMPER_BUTTON) // final
            .onTrue(new InstantCommand(()  -> m_robotSwerveDrive.shiftUp()));
          // .onFalse(new InstantCommand(() -> m_robotSwerveDrive.setToFast()));
        
        new JoystickButton(getDeadbandedDriverController(), XboxController.LEFT_BUMPER_BUTTON) // final
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.shiftDown()));

        new Trigger(() -> getDeadbandedDriverController().getPOV() == 270)
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.shiftDownRot()));

        new Trigger(() -> getDeadbandedDriverController().getPOV() == 90)
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.shiftUpRot()));
        
        
        
       //?  /* Operator Buttons */

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
            .onTrue(i)
            .onFalse(new InstantCommand(() -> m_robotIntake.PIDIn()));
        
        //spins up shooter, no wind down
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

        new Trigger(() -> getDeadbandedOperatorController().getPOV() == 0)
            .onTrue(new InstantCommand(() -> m_robotIntake.ampOuttake(0.5)));

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

        // new Trigger(() -> getVirtualOperatorController().getRawAxis(XboxController.RIGHT_TRIGGER_AXIS) > 0.5)
        //     .onTrue(new InstantCommand(() -> m_robotClimber.climbOut()))
        //     .onFalse(new InstantCommand(() -> m_robotClimber.stopClimb()));

        // new Trigger(() -> getVirtualOperatorController().getRawAxis(XboxController.LEFT_TRIGGER_AXIS) > 0.5)
        //     .onTrue(new InstantCommand(() -> m_robotClimber.climbIn()))
        //     .onFalse(new InstantCommand(() -> m_robotClimber.stopClimb()));

        new Trigger(() -> getVirtualOperatorController().getPOV() == 0)
            .onTrue(new InstantCommand(() -> m_robotIntake.ampOuttake(0.5)));

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

    public VirtualController getVirtualDriverController() {
        return m_virtualDriver;
    }

    public VirtualController getVirtualOperatorController() {
        return m_virtualOperator;
    }
}
