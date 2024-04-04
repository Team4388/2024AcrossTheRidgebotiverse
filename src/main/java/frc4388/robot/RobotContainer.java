/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import org.opencv.video.Video;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc4388.robot.Constants.IntakeConstants;
import frc4388.robot.Constants.OIConstants;
import frc4388.robot.Constants.ShooterConstants;
//import frc4388.robot.commands.Autos.AutoAlign;
import frc4388.robot.commands.Autos.PlaybackChooser;
import frc4388.robot.commands.Swerve.JoystickPlayback;
import frc4388.robot.commands.Swerve.JoystickRecorder;
import frc4388.robot.commands.Swerve.neoJoystickPlayback;
import frc4388.robot.commands.Swerve.neoJoystickRecorder;
import frc4388.robot.commands.Intake.ArmIntakeIn;
import frc4388.robot.commands.Autos.ArmIntakeInAuto;
import frc4388.robot.subsystems.LED;
import frc4388.robot.subsystems.Limelight;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.robot.subsystems.Shooter;
import frc4388.robot.subsystems.Climber;
import frc4388.robot.subsystems.Intake;
import frc4388.utility.DeferredBlock;
import frc4388.utility.configurable.ConfigurableString;
import frc4388.utility.controller.DeadbandedXboxController;
import frc4388.utility.controller.VirtualController;
import frc4388.utility.controller.XboxController;

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

    public final SwerveDrive m_robotSwerveDrive = new SwerveDrive(m_robotMap.leftFront,
                                                                  m_robotMap.rightFront,
                                                                  m_robotMap.leftBack,
                                                                  m_robotMap.rightBack,
                                              
                                                                  m_robotMap.gyro);
    /* Limelight */
    private final Limelight limelight = new Limelight();

    private final Shooter m_robotShooter = new Shooter(m_robotMap.leftShooter, m_robotMap.rightShooter, limelight);

    private final Climber m_robotClimber = new Climber(m_robotMap.climbMotor);

    //private final Intake m_robotIntake = new Intake(m_robotMap.intakeMotor, m_robotMap.pivotMotor);

    /* Controllers */
    private final DeadbandedXboxController m_driverXbox = new DeadbandedXboxController(OIConstants.XBOX_DRIVER_ID);
    private final DeadbandedXboxController m_operatorXbox = new DeadbandedXboxController(OIConstants.XBOX_OPERATOR_ID);    
    private final DeadbandedXboxController m_autoRecorderXbox = new DeadbandedXboxController(2);
    
    private final Intake m_robotIntake = new Intake(m_robotMap.intakeMotor, m_robotMap.pivotMotor);

    /* Virtual Controllers */
    private final VirtualController m_virtualDriver = new VirtualController(0);
    private final VirtualController m_virtualOperator = new VirtualController(1);

    private Command intakeToShootStuff = new ArmIntakeIn(m_robotIntake, m_robotShooter);
    private Command interrupt = new InstantCommand(() -> {}, m_robotIntake, m_robotShooter);

    private SequentialCommandGroup intakeToShoot = new SequentialCommandGroup(
        new InstantCommand(() -> m_robotIntake.talonPIDIn()),
        new InstantCommand(() -> m_robotShooter.idle())
        // new InstantCommand(() -> m_driverXbox.setRumble(RumbleType.kRightRumble, 1.0)).andThen(new WaitCommand(0.2)).andThen(new InstantCommand(() -> m_driverXbox.setRumble(RumbleType.kRightRumble, 0.0))),
        // new InstantCommand(() -> m_robotShooter.spin())
    );

    // private SequentialCommandGroup outtakeToShootFull = new SequentialCommandGroup(
    //     new InstantCommand(() -> m_robotShooter.spin()),
    //     new InstantCommand(() -> m_robotIntake.handoff())
    // );
 
    // private SequentialCommandGroup intakeInToOut = new SequentialCommandGroup(
    //     new InstantCommand(() -> m_robotIntake.rotateArmOut2(), m_robotIntake),
    //     new RunCommand(() -> m_robotIntake.limitNote(), m_robotIntake).until(m_robotIntake.getArmFowardLimitState()),
    //     new InstantCommand(() -> m_robotShooter.spin(), m_robotShooter)
    // );


    // ! Teleop Commands

    //private AutoAlign autoAlign = new AutoAlign(m_robotSwerveDrive, limelight);

    private SequentialCommandGroup autoShoot = new SequentialCommandGroup(
        // MoveToSpeaker,
        //autoAlign,
        new InstantCommand(() -> m_robotShooter.spin()),
        new WaitCommand(3.0),
        new InstantCommand(() -> m_robotIntake.talonHandoff(), m_robotIntake),
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
        new InstantCommand(() -> m_robotIntake.talonHandoff(), m_robotIntake)
    );

    private SequentialCommandGroup turnOffShoot = new SequentialCommandGroup(
        new InstantCommand(() -> m_robotShooter.stop(), m_robotShooter)
        // new InstantCommand(() -> m_robotIntake.talonStopIntakeMotors(), m_robotIntake)
    );

    private SequentialCommandGroup emergencyRetract = new SequentialCommandGroup(
        interrupt,
        new InstantCommand(() -> m_robotIntake.talonPIDIn(), m_robotIntake),
        new InstantCommand(() -> m_robotIntake.talonStopIntakeMotors(), m_robotIntake)
    );

    private SequentialCommandGroup ampShoot = new SequentialCommandGroup(
        new InstantCommand(() -> m_robotIntake.ampPosition()),
        new InstantCommand(() -> m_robotIntake.ampOuttake(0.1)) //TODO: Find Actual Speed
    );

    // ! /*  Autos */
    private Command taxi = new JoystickPlayback(m_robotSwerveDrive, "Taxi.txt"); //new InstantCommand();
    private Command startLeftMoveRight = new InstantCommand(); // new JoystickPlayback(m_robotSwerveDrive, "StartLeftMoveRight.txt");
    private Command startRightMoveLeft = new InstantCommand(); // new JoystickPlayback(m_robotSwerveDrive, "StartRightMoveLeft.txt");
    private String lastAutoName = "final_red_center_4note_taxi.auto";
    private ConfigurableString autoplaybackName = new ConfigurableString("Auto Playback Name", lastAutoName);
    private neoJoystickPlayback autoPlayback = new neoJoystickPlayback(m_robotSwerveDrive, 
    () -> autoplaybackName.get(), // lastAutoName, // () -> autoplaybackName.get(),
           new VirtualController[]{getVirtualDriverController(), getVirtualOperatorController()},
           true, true);


    //Help Simplify Shooting
    // private SequentialCommandGroup pullInArmtoShoot = new SequentialCommandGroup(
    //     new InstantCommand(() -> m_robotIntake.talonPIDIn()),
    //     new InstantCommand(() -> m_robotShooter.spin(), m_robotShooter),
    //     new WaitCommand(1.4).asProxy(),
    //     new InstantCommand(() -> m_robotIntake.talonHandoff(), m_robotIntake),
    //     new WaitCommand(0.5),
    //     new InstantCommand(() -> m_robotShooter.stop(), m_robotShooter),
    //     new InstantCommand(() -> m_robotIntake.talonStopIntakeMotors(), m_robotIntake)
    // );

    private SequentialCommandGroup oneNoteStartingSpeaker = new SequentialCommandGroup (
        new InstantCommand(() -> m_robotShooter.spin(), m_robotShooter),
        new WaitCommand(1).asProxy(),
        new InstantCommand(() -> m_robotIntake.talonHandoff(), m_robotIntake),
        new WaitCommand(1).asProxy(),
        new InstantCommand(() -> m_robotShooter.stop(), m_robotShooter),
        new InstantCommand(() -> m_robotIntake.talonStopIntakeMotors(), m_robotIntake),
        new WaitCommand(1).asProxy(),
        new JoystickPlayback(m_robotSwerveDrive, "Taxi.txt")
    );
    private SequentialCommandGroup oneNoteStartingSpeakerStationary = new SequentialCommandGroup (
        new InstantCommand(() -> m_robotShooter.spin(), m_robotShooter),
        new WaitCommand(1).asProxy(),
        new InstantCommand(() -> m_robotIntake.talonHandoff(), m_robotIntake),
        new WaitCommand(1).asProxy(),
        new InstantCommand(() -> m_robotShooter.stop(), m_robotShooter),
        new InstantCommand(() -> m_robotIntake.talonStopIntakeMotors(), m_robotIntake)
    );
    private SequentialCommandGroup oneNoteStartingFromLeft = new SequentialCommandGroup(
        startLeftMoveRight.asProxy(),
        ejectToShoot.asProxy(),
        taxi.asProxy()
    );
    private SequentialCommandGroup oneNoteStartingFromRight = new SequentialCommandGroup(
        startRightMoveLeft.asProxy(),
        ejectToShoot.asProxy(),
        taxi.asProxy()
    );


    private SequentialCommandGroup twoNoteStartingFromSpeaker = new SequentialCommandGroup(
        new InstantCommand(() -> m_robotShooter.spin(), m_robotShooter),
        new WaitCommand(1).asProxy(),
        new InstantCommand(() -> m_robotIntake.talonHandoff(), m_robotIntake),
        new WaitCommand(1).asProxy(),
        new InstantCommand(() -> m_robotShooter.stop(), m_robotShooter),
        new InstantCommand(() -> m_robotIntake.talonStopIntakeMotors(), m_robotIntake),
        new ArmIntakeInAuto(m_robotIntake, m_robotShooter, m_robotSwerveDrive),
        new InstantCommand(() -> m_robotIntake.talonPIDIn()),
        new InstantCommand(() -> m_robotShooter.spin(), m_robotShooter),
        new WaitCommand(1.4).asProxy(),
        new InstantCommand(() -> m_robotIntake.talonHandoff(), m_robotIntake),
        new WaitCommand(0.5),
        new InstantCommand(() -> m_robotShooter.stop(), m_robotShooter),
        new InstantCommand(() -> m_robotIntake.talonStopIntakeMotors(), m_robotIntake),
        new JoystickPlayback(m_robotSwerveDrive, "Taxi.txt")
        // new WaitCommand(1).asProxy(),
        // new JoystickPlayback(m_robotSwerveDrive, "TwoNotePrt2.txt"),
        // new WaitCommand(0.5).asProxy(),
        // new InstantCommand(() -> m_robotShooter.spin(), m_robotShooter),
        // new WaitCommand(1).asProxy(),
        // new InstantCommand(() -> m_robotIntake.talonHandoff(), m_robotIntake),
        // new WaitCommand(1).asProxy(),
        // new InstantCommand(() -> m_robotShooter.stop(), m_robotShooter),
        // new InstantCommand(() -> m_robotIntake.talonStopIntakeMotors(), m_robotIntake)
    );

    private SequentialCommandGroup stayTwoNoteStartingFromSpeaker = new SequentialCommandGroup(
        new InstantCommand(() -> m_robotShooter.spin(), m_robotShooter),
        new WaitCommand(1).asProxy(),
        new InstantCommand(() -> m_robotIntake.talonHandoff(), m_robotIntake),
        new WaitCommand(1).asProxy(),
        new InstantCommand(() -> m_robotShooter.stop(), m_robotShooter),
        new InstantCommand(() -> m_robotIntake.talonStopIntakeMotors(), m_robotIntake),
        new ArmIntakeInAuto(m_robotIntake, m_robotShooter, m_robotSwerveDrive),
        new InstantCommand(() -> m_robotIntake.talonPIDIn()),
        new InstantCommand(() -> m_robotShooter.spin(), m_robotShooter),
        new WaitCommand(1.4).asProxy(),
        new InstantCommand(() -> m_robotIntake.talonHandoff(), m_robotIntake),
        new WaitCommand(0.5),
        new InstantCommand(() -> m_robotShooter.stop(), m_robotShooter),
        new InstantCommand(() -> m_robotIntake.talonStopIntakeMotors(), m_robotIntake)
    );

    private SequentialCommandGroup threeNoteStartingFromSpeaker = new SequentialCommandGroup(
        new InstantCommand(() -> m_robotShooter.spin(), m_robotShooter),
        new WaitCommand(1).asProxy(),
        new InstantCommand(() -> m_robotIntake.talonHandoff(), m_robotIntake),
        new WaitCommand(1).asProxy(),
        new InstantCommand(() -> m_robotShooter.stop(), m_robotShooter),
        new InstantCommand(() -> m_robotIntake.talonStopIntakeMotors(), m_robotIntake),
        new ArmIntakeInAuto(m_robotIntake, m_robotShooter, m_robotSwerveDrive),
        new InstantCommand(() -> m_robotIntake.talonPIDIn()),
        new InstantCommand(() -> m_robotShooter.spin(), m_robotShooter),
        new WaitCommand(1.4).asProxy(),
        new InstantCommand(() -> m_robotIntake.talonHandoff(), m_robotIntake),
        new WaitCommand(0.5),
        new InstantCommand(() -> m_robotShooter.stop(), m_robotShooter),
        new InstantCommand(() -> m_robotIntake.talonStopIntakeMotors(), m_robotIntake),
        //? Create Another Parallel Command Group :(
        new InstantCommand(() -> m_robotIntake.talonPIDIn()),
        new InstantCommand(() -> m_robotShooter.spin(), m_robotShooter),
        new WaitCommand(1.4).asProxy(),
        new InstantCommand(() -> m_robotIntake.talonHandoff(), m_robotIntake),
        new WaitCommand(0.5),
        new InstantCommand(() -> m_robotShooter.stop(), m_robotShooter),
        new InstantCommand(() -> m_robotIntake.talonStopIntakeMotors(), m_robotIntake),
        new JoystickPlayback(m_robotSwerveDrive, "Taxi.txt")
    );

    private PlaybackChooser playbackChooser = new PlaybackChooser(m_robotSwerveDrive)
            .addOption("Taxi Auto", taxi.asProxy())
            .addOption("One Note Auto Starting in Front of Speaker", oneNoteStartingSpeaker.asProxy())
            .addOption("Stay One Note Auto Starting in Front of Speaker", oneNoteStartingSpeakerStationary.asProxy())
            // .addOption("One Note Auto Starting from Left Position", oneNoteStartingFromLeft.asProxy())
            // .addOption("One Note Auto Starting from Right Position", oneNoteStartingFromRight.asProxy())
            .addOption("Two Note Starting in Front of Speaker", twoNoteStartingFromSpeaker.asProxy())
            .addOption("Stay Two Note Starting in Front of Speaker", stayTwoNoteStartingFromSpeaker.asProxy())
            .buildDisplay();
    
    

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureButtonBindings();        
        configureVirtualButtonBindings();
        // new Trigger(() -> autoplaybackName.get().equals(lastAutoName)).onTrue(new InstantCommand(() -> changeAuto()));
        new DeferredBlock(() -> m_robotSwerveDrive.resetGyro());


            DriverStation.silenceJoystickConnectionWarning(true);
            CameraServer.startAutomaticCapture();

            /* Default Commands */
            // drives the robot with a two-axis input from the driver controller
            // ! Swerve Drive Default Command (Regular Rotation)
            m_robotSwerveDrive.setDefaultCommand(new RunCommand(() -> {
                m_robotSwerveDrive.driveWithInput(getDeadbandedDriverController().getLeft(),
                                                getDeadbandedDriverController().getRight(),
                                    true);
            }, m_robotSwerveDrive)
            .withName("SwerveDrive DefaultCommand"));
           m_robotSwerveDrive.setToSlow();

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

        // ? /* Driver Buttons */

        new JoystickButton(getDeadbandedDriverController(), XboxController.A_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.resetGyroFlip(), m_robotSwerveDrive));

        new JoystickButton(getDeadbandedDriverController(), XboxController.BACK_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.resetGyroRightBlue()))
            .onFalse(new InstantCommand(() -> m_robotSwerveDrive.add180()));

        new JoystickButton(getDeadbandedDriverController(), XboxController.START_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.resetGyroRightAmp()))
            .onFalse(new InstantCommand(() -> m_robotSwerveDrive.add180()));

       // *  /* D-Pad Stuff */
    //    new Trigger(() -> getDeadbandedDriverController().getRawAxis(XboxController.TOP_BOTTOM_DPAD_AXIS) > 0.9)
    //         .onTrue(new InstantCommand(() -> m_robotSwerveDrive.driveWithInput(new Translation2d(0, 1),
    //                                                                            new Translation2d(0, 0),
    //                                                                            true)))
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
                                                                                // true)));
        
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

        // new JoystickButton(getDeadbandedDriverController(), XboxController.RIGHT_BUMPER_BUTTON)
                //    .whileTrue(new JoystickRecorder(m_robotSwerveDrive,
                //                                    () -> getDeadbandedDriverController().getLeftX(),
                //                                    () -> getDeadbandedDriverController().getLeftY(),
                //                                    () -> getDeadbandedDriverController().getRightX(),
                //                                    () -> getDeadbandedDriverController().getRightY(),
                //                                    "Taxi.txt"))
                //    .onFalse(new InstantCommand());

                // new JoystickButton(getDeadbandedDriverController(), XboxController.LEFT_BUMPER_BUTTON)
                //    .onTrue(new JoystickPlayback(m_robotSwerveDrive, "Taxi.txt"))
                //    .onFalse(new InstantCommand()); 
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
        
        // new JoystickButton(getDeadbandedDriverController(), XboxController.Y_BUTTON)
        //     .whileTrue(new InstantCommand(() -> 
        //     m_robotSwerveDrive.driveWithInput(new Translation2d(0, 1),
        //                                       new Translation2d(0, 0),
        //                         true), m_robotSwerveDrive));

        
       //?  /* Operator Buttons */

        new JoystickButton(getDeadbandedOperatorController(), XboxController.Y_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotIntake.talonPIDIn()))
            .onFalse(new InstantCommand(() -> m_robotIntake.talonStopArmMotor()));

        new JoystickButton(getDeadbandedOperatorController(), XboxController.X_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotIntake.talonPIDOut()))
            .onFalse(new InstantCommand(() -> m_robotIntake.talonStopArmMotor()));

        new JoystickButton(getDeadbandedOperatorController(), XboxController.A_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotIntake.talonHandoff()))
            .onFalse(new InstantCommand(() -> m_robotIntake.talonStopIntakeMotors()));
            
        new JoystickButton(getDeadbandedOperatorController(), XboxController.B_BUTTON)
             .onTrue(emergencyRetract);


        // Override Intake Position encoder: out
        new JoystickButton(getDeadbandedOperatorController(), XboxController.BACK_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotIntake.talonSetPivotEncoderPosition(-55), m_robotIntake));

        // Override Intake Position encoder: in
        new JoystickButton(getDeadbandedOperatorController(), XboxController.START_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotIntake.talonSetPivotEncoderPosition(-6.2), m_robotIntake));

        new JoystickButton(getDeadbandedOperatorController(), XboxController.LEFT_BUMPER_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotShooter.spin(0.5), m_robotShooter))
            .onFalse(turnOffShoot);


        new JoystickButton(getDeadbandedOperatorController(), XboxController.RIGHT_BUMPER_BUTTON)
            .onTrue(i)
            .onFalse(new InstantCommand(() -> m_robotIntake.talonPIDIn()));
        
        //spins up shooter, no wind down
        new JoystickButton(getDeadbandedOperatorController(), XboxController.LEFT_JOYSTICK_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotShooter.spin(), m_robotShooter));
        
        // new JoystickButton(getDeadbandedOperatorController(), XboxController.RIGHT_JOYSTICK_BUTTON)
        //     .onTrue(new InstantCommand(() -> m_robotIntake.talonSpinIntakeMotor(), m_robotIntake))
        //     .onFalse(new InstantCommand(() -> m_robotIntake.talonStopIntakeMotors(), m_robotIntake));

        new JoystickButton(getDeadbandedOperatorController(), XboxController.RIGHT_JOYSTICK_BUTTON)
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

   private void configureVirtualButtonBindings() {
        
        // ? /* Driver Buttons */

        new JoystickButton(getVirtualDriverController(), XboxController.A_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.resetGyroFlip(), m_robotSwerveDrive));

        new JoystickButton(getVirtualDriverController(), XboxController.BACK_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.resetGyroRightBlue()))
            .onFalse(new InstantCommand(() -> m_robotSwerveDrive.add180()));

        new JoystickButton(getVirtualDriverController(), XboxController.START_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.resetGyroRightAmp()))
            .onFalse(new InstantCommand(() -> m_robotSwerveDrive.add180()));


       // *  /* D-Pad Stuff */
    //    new Trigger(() -> getVirtualDriverController().getRawAxis(XboxController.TOP_BOTTOM_DPAD_AXIS) > 0.9)
    //         .onTrue(new InstantCommand(() -> m_robotSwerveDrive.driveWithInput(new Translation2d(0, 1),
    //                                                                            new Translation2d(0, 0),
    //                                                                            true)))
    //         .onFalse(new InstantCommand(() -> m_robotSwerveDrive.driveWithInput(new Translation2d(0, 0), 
    //                                                                             new Translation2d(0, 0), 
    //                                                                             true)));

    //    new Trigger(() -> getVirtualDriverController().getRawAxis(XboxController.TOP_BOTTOM_DPAD_AXIS) > -0.9)
    //         .onTrue(new InstantCommand(() -> m_robotSwerveDrive.driveWithInput(new Translation2d(0, -1),
    //                                                                            new Translation2d(0, 0),
    //                                                                            true)))
    //         .onFalse(new InstantCommand(() -> m_robotSwerveDrive.driveWithInput(new Translation2d(0, 0), 
    //                                                                             new Translation2d(0, 0), 
    //                                                                             true)));

    //    new Trigger(() -> getVirtualDriverController().getRawAxis(XboxController.LEFT_RIGHT_DPAD_AXIS) > 0.9)
    //         .onTrue(new InstantCommand(() -> m_robotSwerveDrive.driveWithInput(new Translation2d(1, 0),
    //                                                                            new Translation2d(0, 0),
    //                                                                            true)))
    //         .onFalse(new InstantCommand(() -> m_robotSwerveDrive.driveWithInput(new Translation2d(0, 0), 
    //                                                                             new Translation2d(0, 0), 
    //                                                                             true)));

    //    new Trigger(() -> getVirtualDriverController().getRawAxis(XboxController.LEFT_RIGHT_DPAD_AXIS) > -0.9)
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

        // new JoystickButton(getDeadbandedDriverController(), XboxController.RIGHT_BUMPER_BUTTON)
                //    .whileTrue(new JoystickRecorder(m_robotSwerveDrive,
                //                                    () -> getDeadbandedDriverController().getLeftX(),
                //                                    () -> getDeadbandedDriverController().getLeftY(),
                //                                    () -> getDeadbandedDriverController().getRightX(),
                //                                    () -> getDeadbandedDriverController().getRightY(),
                //                                    "Taxi.txt"))
                //    .onFalse(new InstantCommand());

                // new JoystickButton(getDeadbandedDriverController(), XboxController.LEFT_BUMPER_BUTTON)
                //    .onTrue(new JoystickPlayback(m_robotSwerveDrive, "Taxi.txt"))
                //    .onFalse(new InstantCommand()); 
        // ! /* Speed */
        new JoystickButton(getVirtualDriverController(), XboxController.RIGHT_BUMPER_BUTTON) // final
            .onTrue(new InstantCommand(()  -> m_robotSwerveDrive.shiftUp()));
          // .onFalse(new InstantCommand(() -> m_robotSwerveDrive.setToFast()));
        
        new JoystickButton(getVirtualDriverController(), XboxController.LEFT_BUMPER_BUTTON) // final
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.shiftDown()));

        new Trigger(() -> getVirtualDriverController().getPOV() == 270)
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.shiftDownRot()));

        new Trigger(() -> getVirtualDriverController().getPOV() == 90)
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.shiftUpRot()));
        
        // new JoystickButton(getVirtualDriverController(), XboxController.Y_BUTTON)
        //     .whileTrue(new InstantCommand(() -> 
        //     m_robotSwerveDrive.driveWithInput(new Translation2d(0, 1),
        //                                       new Translation2d(0, 0),
        //                         true), m_robotSwerveDrive));

        
       //?  /* Operator Buttons */

        new JoystickButton(getVirtualOperatorController(), XboxController.Y_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotIntake.talonPIDIn()))
            .onFalse(new InstantCommand(() -> m_robotIntake.talonStopArmMotor()));

        new JoystickButton(getVirtualOperatorController(), XboxController.X_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotIntake.talonPIDOut()))
            .onFalse(new InstantCommand(() -> m_robotIntake.talonStopArmMotor()));

        new JoystickButton(getVirtualOperatorController(), XboxController.A_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotIntake.talonHandoff()))
            .onFalse(new InstantCommand(() -> m_robotIntake.talonStopIntakeMotors()));
            
        new JoystickButton(getVirtualOperatorController(), XboxController.B_BUTTON)
             .onTrue(emergencyRetract);


        // Override Intake Position encoder: out
        new JoystickButton(getVirtualOperatorController(), XboxController.BACK_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotIntake.talonSetPivotEncoderPosition(-55), m_robotIntake));

        // Override Intake Position encoder: in
        new JoystickButton(getVirtualOperatorController(), XboxController.START_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotIntake.talonSetPivotEncoderPosition(-6.2), m_robotIntake));

        new JoystickButton(getVirtualOperatorController(), XboxController.LEFT_BUMPER_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotShooter.spin(0.5), m_robotShooter))
            .onFalse(turnOffShoot);


        new JoystickButton(getVirtualOperatorController(), XboxController.RIGHT_BUMPER_BUTTON)
            .onTrue(i)
            .onFalse(new InstantCommand(() -> m_robotIntake.talonPIDIn()));
        
        //spins up shooter, no wind down
        new JoystickButton(getVirtualOperatorController(), XboxController.LEFT_JOYSTICK_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotShooter.spin(), m_robotShooter));
        
        // new JoystickButton(getDeadbandedOperatorController(), XboxController.RIGHT_JOYSTICK_BUTTON)
        //     .onTrue(new InstantCommand(() -> m_robotIntake.talonSpinIntakeMotor(), m_robotIntake))
        //     .onFalse(new InstantCommand(() -> m_robotIntake.talonStopIntakeMotors(), m_robotIntake));

        new JoystickButton(getVirtualOperatorController(), XboxController.RIGHT_JOYSTICK_BUTTON)
            .onTrue(emergencyRetract);

        new Trigger(() -> getVirtualOperatorController().getRawAxis(XboxController.RIGHT_TRIGGER_AXIS) > 0.5)
            .onTrue(new InstantCommand(() -> m_robotClimber.climbOut()))
            .onFalse(new InstantCommand(() -> m_robotClimber.stopClimb()));

        new Trigger(() -> getVirtualOperatorController().getRawAxis(XboxController.LEFT_TRIGGER_AXIS) > 0.5)
            .onTrue(new InstantCommand(() -> m_robotClimber.climbIn()))
            .onFalse(new InstantCommand(() -> m_robotClimber.stopClimb()));

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
