/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import java.time.Instant;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc4388.robot.Constants.OIConstants;
import frc4388.robot.commands.Autos.AutoAlign;
import frc4388.robot.commands.Autos.PlaybackChooser;
import frc4388.robot.commands.Swerve.JoystickPlayback;
import frc4388.robot.commands.Swerve.JoystickRecorder;
import frc4388.robot.commands.Intake.ArmIntakeIn;
import frc4388.robot.commands.Intake.RotateIntakeToPosition;
import frc4388.robot.subsystems.LED;
import frc4388.robot.subsystems.Limelight;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.robot.subsystems.Shooter;
import frc4388.robot.subsystems.Intake;
import frc4388.utility.controller.DeadbandedXboxController;
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
    
     private final Shooter m_robotShooter = new Shooter(m_robotMap.leftShooter, m_robotMap.rightShooter);

     private final Intake m_robotIntake = new Intake(m_robotMap.intakeMotor, m_robotMap.pivotMotor);

    //private final Intake m_robotIntake = new Intake(m_robotMap.intakeMotor, m_robotMap.pivotMotor);

    /* Controllers */
    private final DeadbandedXboxController m_driverXbox = new DeadbandedXboxController(OIConstants.XBOX_DRIVER_ID);
    private final DeadbandedXboxController m_operatorXbox = new DeadbandedXboxController(OIConstants.XBOX_OPERATOR_ID);
    
    /* Autos */
    private Limelight limelight = new Limelight();
    private AutoAlign autoAlign = new AutoAlign(m_robotSwerveDrive, limelight);

    private Command taxi = new JoystickPlayback(m_robotSwerveDrive, "Taxi.txt");
    private Command MoveToSpeaker = new JoystickPlayback(m_robotSwerveDrive, "MoveToSpeaker.txt");
    private Command intakeToShootStuff = new ArmIntakeIn(m_robotIntake, m_robotShooter);

    private SequentialCommandGroup intakeToShoot = new SequentialCommandGroup(
        new InstantCommand(() -> m_robotIntake.pidIn()),
        new InstantCommand(() -> m_robotShooter.spin())
    );

    private SequentialCommandGroup outtakeToShootFull = new SequentialCommandGroup(
        new InstantCommand(() -> m_robotShooter.spin()),
        new InstantCommand(() -> m_robotIntake.handoff())
    );
 
    private SequentialCommandGroup intakeInToOut = new SequentialCommandGroup(
        new InstantCommand(() -> m_robotIntake.rotateArmOut2(), m_robotIntake),
        new RunCommand(() -> m_robotIntake.limitNote(), m_robotIntake).until(m_robotIntake.getArmFowardLimitState()),
        new InstantCommand(() -> m_robotShooter.spin(), m_robotShooter)
    );

    private SequentialCommandGroup autoShoot = new SequentialCommandGroup(
        new RunCommand(() -> MoveToSpeaker.execute()),
        new RunCommand(() -> autoAlign.execute())
    );


    private SequentialCommandGroup i = new SequentialCommandGroup(
        intakeToShootStuff, intakeToShoot
    );

    private SequentialCommandGroup ejectToShoot = new SequentialCommandGroup(
        new InstantCommand(() -> m_robotShooter.spin(), m_robotShooter),
        new InstantCommand(() -> m_robotIntake.handoff(), m_robotIntake)
    );

    private SequentialCommandGroup turnOffShoot = new SequentialCommandGroup(
        new InstantCommand(() -> m_robotShooter.stop(), m_robotShooter),
        new InstantCommand(() -> m_robotIntake.stopIntakeMotors(), m_robotIntake)
    );

    

    

    private PlaybackChooser playbackChooser = new PlaybackChooser(m_robotSwerveDrive)
            .addOption("Taxi Auto", new JoystickPlayback(m_robotSwerveDrive, "Taxi.txt"))
            .buildDisplay();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureButtonBindings();

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

        
        SmartDashboard.putNumber("Velocity Output", m_robotIntake.getVelocity());

       // m_robotIntake.resetPostion();
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
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.resetGyro(), m_robotSwerveDrive));
        
        
        
        /* Auto Recording */
        // new JoystickButton(getDeadbandedDriverController(), XboxController.RIGHT_BUMPER_BUTTON)
        //    .whileTrue(new JoystickRecorder(m_robotSwerveDrive,
        //                                    () -> getDeadbandedDriverController().getLeftX(),
        //                                    () -> getDeadbandedDriverController().getLeftY(),
        //                                    () -> getDeadbandedDriverController().getRightX(),
        //                                    () -> getDeadbandedDriverController().getRightY(),
        //                                    "IntenseTaxi.txt"))
        //    .onFalse(new InstantCommand());

        new JoystickButton(getDeadbandedDriverController(), XboxController.X_BUTTON)
           .onTrue(new JoystickPlayback(m_robotSwerveDrive, "Taxi.txt"))
           .onFalse(new InstantCommand()); 
        
        /* Speed */
        new JoystickButton(getDeadbandedDriverController(), XboxController.RIGHT_BUMPER_BUTTON) // final
            .onTrue(new InstantCommand(()  -> m_robotSwerveDrive.shiftUp()));
         //   .onFalse(new InstantCommand(() -> m_robotSwerveDrive.setToFast()));
        
        new JoystickButton(getDeadbandedDriverController(), XboxController.LEFT_BUMPER_BUTTON) // final
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.shiftDown()));
        
        
        
        /* Operator Buttons */
        // new JoystickButton(getDeadbandedDriverController(), XboxController.B_BUTTON)
        //      .onTrue(new InstantCommand(() -> m_robotIntake.spinIntakeMotor(), m_robotIntake))
        //      .onFalse(new InstantCommand(() -> m_robotIntake.stopIntakeMotors(), m_robotIntake));
             
        // new JoystickButton(getDeadbandedOperatorController(), XboxController.A_BUTTON)
        //      .onTrue(new InstantCommand(() ->  new RotateIntakeToPosition(m_robotIntake, 360).execute(), m_robotIntake))
        //      .onFalse(new InstantCommand(() -> new RotateIntakeToPosition(m_robotIntake, 0).execute(), m_robotShooter));

        new JoystickButton(getDeadbandedOperatorController(), XboxController.Y_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotIntake.pidIn()))
            .onFalse(new InstantCommand(() -> m_robotIntake.stopArmMotor()));

        new JoystickButton(getDeadbandedOperatorController(), XboxController.X_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotIntake.pidOut()))
            .onFalse(new InstantCommand(() -> m_robotIntake.stopArmMotor()));

        // //Pull arm in
        // new JoystickButton(getDeadbandedOperatorController(), XboxController.RIGHT_BUMPER_BUTTON)
        //     .onTrue(new InstantCommand(() -> m_robotIntake.pidIn(), m_robotIntake))
        //     .onFalse(new InstantCommand(() -> m_robotIntake.stopArmMotor(), m_robotIntake));

        // //Pull arm out
        // new JoystickButton(getDeadbandedOperatorController(), XboxController.LEFT_BUMPER_BUTTON)
        //     .onTrue(new InstantCommand(() -> m_robotIntake.pidOut(), m_robotIntake))
        //     .onFalse(new InstantCommand(() -> m_robotIntake.stopArmMotor(), m_robotIntake));
        
        // //Intake Note
        // new JoystickButton(getDeadbandedOperatorController(), XboxController.B_BUTTON)
        //     .onTrue(new InstantCommand(() -> m_robotIntake.spinIntakeMotor(), m_robotIntake))
        //     .onFalse(new InstantCommand(() -> m_robotIntake.stopIntakeMotors(), m_robotIntake));
        
        // //Outtake Note
        // new JoystickButton(getDeadbandedOperatorController(), XboxController.Y_BUTTON)
        //     .onTrue(new InstantCommand(() -> m_robotIntake.handoff(), m_robotIntake))
        //     .onFalse(new InstantCommand(() -> m_robotIntake.stopIntakeMotors(), m_robotIntake));
            
        //Spin Shooter Motors
        new JoystickButton(getDeadbandedOperatorController(), XboxController.A_BUTTON)
             .onTrue(new InstantCommand(() -> m_robotShooter.spin(), m_robotShooter))
             .onFalse(new InstantCommand(() -> m_robotShooter.stop(), m_robotShooter));

        // //Intake Note and ramp up shooter to 40%
        // new JoystickButton(getDeadbandedOperatorController(), XboxController.A_BUTTON)
        //     .onTrue(intakeToShoot);
        
        // //Ramps up shooter to 100% to Shooter
        // new JoystickButton(getDeadbandedOperatorController(), XboxController.B_BUTTON)
        //     .onTrue(outtakeToShootFull);
        

        new JoystickButton(getDeadbandedOperatorController(), XboxController.LEFT_BUMPER_BUTTON)
            .onTrue(ejectToShoot)
            .onFalse(turnOffShoot);


        new JoystickButton(getDeadbandedOperatorController(), XboxController.RIGHT_BUMPER_BUTTON)
            .onTrue(i)
            .onFalse(new InstantCommand(() -> m_robotIntake.pidIn()));
        
        
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
         //no auto
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
}
