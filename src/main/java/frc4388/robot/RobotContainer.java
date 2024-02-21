/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc4388.robot.Constants.OIConstants;
import frc4388.robot.commands.Autos.AutoAlign;
import frc4388.robot.commands.Autos.PlaybackChooser;
import frc4388.robot.commands.Swerve.JoystickPlayback;
import frc4388.robot.commands.Swerve.JoystickRecorder;
import frc4388.robot.commands.Swerve.neoJoystickPlayback;
import frc4388.robot.commands.Swerve.neoJoystickRecorder;
import frc4388.robot.commands.Intake.ArmIntakeIn;
import frc4388.robot.subsystems.LED;
import frc4388.robot.subsystems.Limelight;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.robot.subsystems.Shooter;
import frc4388.robot.subsystems.Intake;
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

    private final Shooter m_robotShooter = new Shooter(m_robotMap.leftShooter, m_robotMap.rightShooter);

    private final Intake m_robotIntake = new Intake(m_robotMap.intakeMotor, m_robotMap.pivotMotor);

    //private final Intake m_robotIntake = new Intake(m_robotMap.intakeMotor, m_robotMap.pivotMotor);

    /* Controllers */
    private final DeadbandedXboxController m_driverXbox = new DeadbandedXboxController(OIConstants.XBOX_DRIVER_ID);
    private final DeadbandedXboxController m_operatorXbox = new DeadbandedXboxController(OIConstants.XBOX_OPERATOR_ID);    
    private final DeadbandedXboxController m_autoRecorderXbox = new DeadbandedXboxController(2);


    /* Virtual Controllers */
    private final VirtualController m_virtualDriver = new VirtualController(0);
    private final VirtualController m_virtualOperator = new VirtualController(1);

    private Command intakeToShootStuff = new ArmIntakeIn(m_robotIntake, m_robotShooter);

    private SequentialCommandGroup intakeToShoot = new SequentialCommandGroup(
        new InstantCommand(() -> m_robotIntake.talonPIDIn())
        //new InstantCommand(() -> m_robotShooter.spin())
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


    private AutoAlign autoAlign = new AutoAlign(m_robotSwerveDrive, limelight);

    private SequentialCommandGroup autoShoot = new SequentialCommandGroup(
        // MoveToSpeaker,
        autoAlign,
        new InstantCommand(() -> m_robotShooter.spin()),
        new WaitCommand(3.0),
        new InstantCommand(() -> m_robotIntake.talonHandoff(), m_robotIntake),
        new WaitCommand(3.0),
        new InstantCommand(() -> m_robotShooter.idle()),
        new InstantCommand(() -> autoAlign.reverse()),
        autoAlign.asProxy()
    );


    private SequentialCommandGroup i = new SequentialCommandGroup(
        intakeToShootStuff, intakeToShoot
    );

    private SequentialCommandGroup ejectToShoot = new SequentialCommandGroup(
         new InstantCommand(() -> m_robotShooter.spin(), m_robotShooter)
        // new WaitCommand(0.75),
        //new InstantCommand(() -> m_robotIntake.talonHandoff(), m_robotIntake)
    );

    private SequentialCommandGroup turnOffShoot = new SequentialCommandGroup(
        new InstantCommand(() -> m_robotShooter.stop(), m_robotShooter),
        new InstantCommand(() -> m_robotIntake.talonStopIntakeMotors(), m_robotIntake)
    );

    /* Autos */
    private Command taxi = new InstantCommand(); // new JoystickPlayback(m_robotSwerveDrive, "Taxi.txt");
    private Command startLeftMoveRight = new InstantCommand(); // new JoystickPlayback(m_robotSwerveDrive, "StartLeftMoveRight.txt");
    private Command startRightMoveLeft = new InstantCommand(); // new JoystickPlayback(m_robotSwerveDrive, "StartRightMoveLeft.txt");

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
        intakeToShootStuff.asProxy(),
        new WaitCommand(1).asProxy(),
        new JoystickPlayback(m_robotSwerveDrive, "TwoNotePrt1.txt"),
        intakeToShoot.asProxy(),
        new WaitCommand(1).asProxy(),
        new JoystickPlayback(m_robotSwerveDrive, "TwoNotePrt2.txt"),
        new WaitCommand(0.5).asProxy(),
        new InstantCommand(() -> m_robotShooter.spin(), m_robotShooter),
        new WaitCommand(1).asProxy(),
        new InstantCommand(() -> m_robotIntake.talonHandoff(), m_robotIntake),
        new WaitCommand(1).asProxy(),
        new InstantCommand(() -> m_robotShooter.stop(), m_robotShooter),
        new InstantCommand(() -> m_robotIntake.talonStopIntakeMotors(), m_robotIntake)
    );
    private PlaybackChooser playbackChooser = new PlaybackChooser(m_robotSwerveDrive)
            .addOption("Taxi Auto", taxi.asProxy())
            .addOption("One Note Auto Starting in Front of Speaker", oneNoteStartingSpeaker)
            .addOption("One Note Auto Starting in Front of Speaker, But Stay", oneNoteStartingSpeakerStationary)
            .addOption("One Note Auto Starting from Left Position", oneNoteStartingFromLeft)
            .addOption("One Note Auto Starting from Right Position", oneNoteStartingFromRight)
            .buildDisplay();
    
    

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureButtonBindings();        
       // configureVirtualButtonBindings();


        DriverStation.silenceJoystickConnectionWarning(true);
        CameraServer.startAutomaticCapture();

        /* Default Commands */
        // drives the robot with a two-axis input from the driver controller
        m_robotSwerveDrive.setDefaultCommand(new RunCommand(() -> {
            m_robotSwerveDrive.driveWithInput(getDeadbandedDriverController().getLeft(),
                                              getDeadbandedDriverController().getRight(),
                                true);
        }, m_robotSwerveDrive)
        .withName("SwerveDrive DefaultCommand"));
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
        /* Driver Buttons */
        new JoystickButton(getDeadbandedDriverController(), XboxController.A_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.resetGyroFlip(), m_robotSwerveDrive));
        
        /* Auto Recording */
        new JoystickButton(getDeadbandedDriverController(), XboxController.RIGHT_BUMPER_BUTTON)
           .whileTrue(new JoystickRecorder(m_robotSwerveDrive, m_robotShooter, m_robotIntake,
                                           () -> getDeadbandedDriverController().getLeftX(),
                                           () -> getDeadbandedDriverController().getLeftY(),
                                           () -> getDeadbandedDriverController().getRightX(),
                                           () -> getDeadbandedDriverController().getRightY(),
                                           () -> getDeadbandedOperatorController().getLeftBumper(),
                                           () -> getDeadbandedOperatorController().getRightBumper(),
                                           "Taxi.txt"))
           .onFalse(new InstantCommand());

        new JoystickButton(getDeadbandedDriverController(), XboxController.LEFT_BUMPER_BUTTON)
           .onTrue(new JoystickPlayback(m_robotSwerveDrive, "Taxi.txt"))
           .onFalse(new InstantCommand()); 

        // new JoystickButton(m_autoRecorderXbox, XboxController.LEFT_BUMPER_BUTTON)
        //    .whileTrue(new neoJoystickRecorder(m_robotSwerveDrive,
        //                 new DeadbandedXboxController[]{getDeadbandedDriverController(), getDeadbandedOperatorController()},
        //                                    "2note.auto"))
        //    .onFalse(new InstantCommand());
        
        // new JoystickButton(m_autoRecorderXbox, XboxController.RIGHT_BUMPER_BUTTON)
        //    .onTrue(new neoJoystickPlayback(m_robotSwerveDrive,
        //    "2note.auto",
        //    new VirtualController[]{getVirtualDriverController(), getVirtualOperatorController()},
        //    true, false))
        //    .onFalse(new InstantCommand());

        // /* Speed */
        new JoystickButton(getDeadbandedDriverController(), XboxController.RIGHT_BUMPER_BUTTON) // final
            .onTrue(new InstantCommand(()  -> m_robotSwerveDrive.shiftUp()));
         //   .onFalse(new InstantCommand(() -> m_robotSwerveDrive.setToFast()));
        
        new JoystickButton(getDeadbandedDriverController(), XboxController.LEFT_BUMPER_BUTTON) // final
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.shiftDown()));
        
        
        
        /* Operator Buttons */

        new JoystickButton(getDeadbandedOperatorController(), XboxController.Y_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotIntake.talonPIDIn()))
            .onFalse(new InstantCommand(() -> m_robotIntake.talonStopArmMotor()));

        new JoystickButton(getDeadbandedOperatorController(), XboxController.B_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotIntake.talonPIDOut()))
            .onFalse(new InstantCommand(() -> m_robotIntake.talonStopArmMotor()));

        new JoystickButton(getDeadbandedOperatorController(), XboxController.A_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotIntake.talonHandoff()))
            .onFalse(new InstantCommand(() -> m_robotIntake.talonStopIntakeMotors()));


        // Override Intake Position encoder: out
        new JoystickButton(getDeadbandedOperatorController(), XboxController.BACK_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotIntake.talonSetPivotEncoderPosition(-57), m_robotIntake));

        // Override Intake Position encoder: in
        new JoystickButton(getDeadbandedOperatorController(), XboxController.START_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotIntake.talonSetPivotEncoderPosition(0), m_robotIntake));

        //Spin Shooter Motors
        new JoystickButton(getDeadbandedOperatorController(), XboxController.X_BUTTON)
             .onTrue(new InstantCommand(() -> m_robotShooter.spin(), m_robotShooter))
             .onFalse(new InstantCommand(() -> m_robotShooter.stop(), m_robotShooter));

        new JoystickButton(getDeadbandedOperatorController(), XboxController.LEFT_BUMPER_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotShooter.spin(), m_robotShooter))
            .onFalse(turnOffShoot);


        new JoystickButton(getDeadbandedOperatorController(), XboxController.RIGHT_BUMPER_BUTTON)
            .onTrue(i)
            .onFalse(new InstantCommand(() -> m_robotIntake.talonPIDIn()));
        
        //spins up shooter, no wind down
        new JoystickButton(getDeadbandedOperatorController(), XboxController.LEFT_JOYSTICK_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotShooter.spin(), m_robotShooter));
        
    }

   private void configureVirtualButtonBindings() {
        /* Driver Buttons */
        new JoystickButton(getVirtualDriverController(), XboxController.A_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.resetGyro(), m_robotSwerveDrive));
        
        /* Speed */
        new JoystickButton(getVirtualDriverController(), XboxController.RIGHT_BUMPER_BUTTON) // final
            .onTrue(new InstantCommand(()  -> m_robotSwerveDrive.shiftUp()));
         //   .onFalse(new InstantCommand(() -> m_robotSwerveDrive.setToFast()));
        
        new JoystickButton(getVirtualDriverController(), XboxController.LEFT_BUMPER_BUTTON) // final
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.shiftDown()));
        
        
        
        /* Operator Buttons */

        new JoystickButton(getVirtualOperatorController(), XboxController.Y_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotIntake.talonPIDIn()))
            .onFalse(new InstantCommand(() -> m_robotIntake.talonStopArmMotor()));

        new JoystickButton(getVirtualOperatorController(), XboxController.A_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotIntake.talonPIDOut()))
            .onFalse(new InstantCommand(() -> m_robotIntake.talonStopArmMotor()));


        // Override Intake Position encoder: out
        new JoystickButton(getVirtualOperatorController(), XboxController.BACK_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotIntake.talonSetPivotEncoderPosition(-53), m_robotIntake));

        // Override Intake Position encoder: in
        new JoystickButton(getVirtualOperatorController(), XboxController.START_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotIntake.talonSetPivotEncoderPosition(0), m_robotIntake));

        //Spin Shooter Motors
        new JoystickButton(getVirtualOperatorController(), XboxController.X_BUTTON)
             .onTrue(new InstantCommand(() -> m_robotShooter.spin(), m_robotShooter))
             .onFalse(new InstantCommand(() -> m_robotShooter.stop(), m_robotShooter));

        new JoystickButton(getVirtualOperatorController(), XboxController.LEFT_BUMPER_BUTTON)
            .onTrue(ejectToShoot)
            .onFalse(turnOffShoot);


        new JoystickButton(getVirtualOperatorController(), XboxController.RIGHT_BUMPER_BUTTON)
            .onTrue(i)
            .onFalse(new InstantCommand(() -> m_robotIntake.talonPIDIn()));
        
        
        new JoystickButton(getDeadbandedOperatorController(), XboxController.LEFT_JOYSTICK_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotShooter.spin(), m_robotShooter));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        //no auto
        // return new neoJoystickPlayback(m_robotSwerveDrive,
        //    "2note.auto",
        //    new VirtualController[]{getVirtualDriverController(), getVirtualOperatorController()},
        //    true, false);
        return playbackChooser.getCommand();
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
