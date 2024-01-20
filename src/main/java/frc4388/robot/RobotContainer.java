/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc4388.robot.Constants.OIConstants;
import frc4388.robot.commands.Autos.PlaybackChooser;
import frc4388.robot.commands.Swerve.JoystickPlayback;
import frc4388.robot.commands.Swerve.JoystickRecorder;
import frc4388.robot.subsystems.LED;
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
    private final LED m_robotLED = new LED(m_robotMap.LEDController);

    public final SwerveDrive m_robotSwerveDrive = new SwerveDrive(m_robotMap.leftFront,
                                                                  m_robotMap.rightFront,
                                                                  m_robotMap.leftBack,
                                                                  m_robotMap.rightBack,
                                                                  m_robotMap.gyro);
    
    // private final Shooter m_robotShooter = new Shooter(m_robotMap.leftShooter, m_robotMap.rightShooter);
    
    //private final Intake m_robotIntake = new Intake(m_robotMap.intakeMotor);

    /* Controllers */
    private final DeadbandedXboxController m_driverXbox = new DeadbandedXboxController(OIConstants.XBOX_DRIVER_ID);
    private final DeadbandedXboxController m_operatorXbox = new DeadbandedXboxController(OIConstants.XBOX_OPERATOR_ID);

    private Command taxi = new JoystickPlayback(m_robotSwerveDrive, "Taxi.txt");

    private PlaybackChooser playbackChooser;

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

        playbackChooser = new PlaybackChooser(m_robotSwerveDrive)
            .addOption("Taxi Auto", new JoystickPlayback(m_robotSwerveDrive, "Taxi.txt"))
            .buildDisplay();
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
        //     .whileTrue(new JoystickRecorder(m_robotSwerveDrive,
        //                                     () -> getDeadbandedDriverController().getLeftX(),
        //                                     () -> getDeadbandedDriverController().getLeftY(),
        //                                     () -> getDeadbandedDriverController().getRightX(),
        //                                     () -> getDeadbandedDriverController().getRightY(),
        //                                     "Taxi.txt"))
        //     .onFalse(new InstantCommand());

        // new JoystickButton(getDeadbandedDriverController(), XboxController.LEFT_BUMPER_BUTTON)
        //     .onTrue(new JoystickPlayback(m_robotSwerveDrive, "Taxi.txt"))
        //     .onFalse(new InstantCommand()); 
        
        /* Speed */
        new JoystickButton(getDeadbandedDriverController(), XboxController.RIGHT_BUMPER_BUTTON) // final
            .onTrue(new InstantCommand(()  -> m_robotSwerveDrive.setToTurbo()))
            .onFalse(new InstantCommand(() -> m_robotSwerveDrive.setToFast()));
        
        new JoystickButton(getDeadbandedDriverController(), XboxController.LEFT_BUMPER_BUTTON) // final
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.setToSlow()));
        
        // new JoystickButton(getDeadbandedDriverController(), XboxController.B_BUTTON)
        //     .onTrue(new InstantCommand(() -> m_robotShooter.spin(), m_robotShooter))
        //     .onFalse(new InstantCommand(() -> m_robotShooter.stop(), m_robotShooter));
        
        /* Operator Buttons */
        
        
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // no auto
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
