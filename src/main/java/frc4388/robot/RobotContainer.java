/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot;

import java.io.File;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Commands
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc4388.robot.commands.Swerve.StayInPosition;
import frc4388.robot.constants.Constants;
import frc4388.robot.constants.Constants.OIConstants;
import frc4388.robot.constants.Constants.SimConstants.Mode;
import frc4388.robot.subsystems.swerve.SimpleSwerveSim;
import frc4388.robot.subsystems.swerve.SwerveDrive;
import frc4388.robot.subsystems.vision.Vision;
import frc4388.utility.DeferredBlock;
import frc4388.utility.compute.FieldPositions;
import frc4388.utility.compute.TimesNegativeOne;
import frc4388.utility.controller.DeadbandedXboxController;
// Autos
import frc4388.utility.controller.VirtualController;
import frc4388.utility.controller.XboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (2including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* RobotMap */
    
    public final RobotMap m_robotMap = new RobotMap(RobotBase.isReal() ? Mode.REAL : Mode.SIM);
    
    /*Limit Switch */
    // public final DigitalInput m_armLimitSwitch = new DigitalInput(9);

    /* Subsystems */
    // public final Lidar m_lidar = new Lidar();
    // public final LED m_robotLED = new LED(Constants.LEDConstants.LED_SPARK_ID);
    public final SimpleSwerveSim m_robotSwerveSIM = new SimpleSwerveSim();
    //Testing of Colors
    public final Vision m_vision = new Vision();
    public final SwerveDrive m_robotSwerveDrive = new SwerveDrive(m_robotMap.swerveDrivetrain, m_vision);
    // public final Intake m_robotIntake = new Intake(m_robotMap.intakeIO, m_robotSwerveDrive);
    // public final Shooter m_robotShooter = new Shooter(m_robotMap.shooterIO, m_robotSwerveDrive, m_robotIntake, m_robotLED);
    

    /* Controllers */
    private final DeadbandedXboxController m_driverXbox   = new DeadbandedXboxController(OIConstants.XBOX_DRIVER_ID);
    private final DeadbandedXboxController m_operatorXbox = new DeadbandedXboxController(OIConstants.XBOX_OPERATOR_ID);


    // private final ButtonBox m_buttonBox = new ButtonBox(OIConstants.BUTTONBOX_ID);

    // public List<Subsystem> subsystems = new ArrayList<>();
    private final StayInPosition m_stayInPosition = new StayInPosition(m_robotSwerveDrive);
    
    private Pose2d currentPose = new Pose2d(0, 0, new Rotation2d());
        // ! Teleop Commands
        public void stop() {
            new InstantCommand(()->{}, m_robotSwerveDrive).schedule();
            m_robotSwerveDrive.stopModules();
            Constants.AutoConstants.Y_OFFSET_TRIM.set(0);
        }
    
        // ! /*  Autos */
        private SendableChooser<String> autoChooser;
        private Command autoCommand;
     

  
    
        
        public RobotContainer() {
            
            configureSINGLEBindings();
            
            // Called on first robot enable
            DeferredBlock.addBlock(() -> {
                m_robotSwerveDrive.resetGyro();
            }, false);
    
            // Called on every robot enable
            DeferredBlock.addBlock(() -> {
                // m_robotIntake.setMode(IntakeMode.Idle);
                // m_robotShooter.spinUpIdle();
                // m_robotIntake.io.updateGains();
                TimesNegativeOne.update();
                FieldPositions.update();
                // m_robotShooter.io.updateGains();
            }, true);

            DriverStation.silenceJoystickConnectionWarning(true);
    
            // Drive normally
            m_robotSwerveDrive.setDefaultCommand(new RunCommand(() -> {
                m_robotSwerveDrive.driveWithInput(
                    getDeadbandedDriverController().getLeft(),
                    getDeadbandedDriverController().getRight(),true);
    
            }, m_robotSwerveDrive)
            .withName("SwerveDrive DefaultCommand"));
            
            m_robotSwerveDrive.setToSlow();
            
            makeAutoChooser();
            SmartDashboard.putData("Auto Chooser", autoChooser);
    
        }
        
    

   private boolean lt_down() {
        return getDeadbandedDriverController().getLeftTriggerAxis() > 0.8;
   }

   private boolean rt_down() {
        return getDeadbandedDriverController().getRightTriggerAxis() > 0.8;
   }

   private void configureSINGLEBindings() {

        String controllerInstructions = "" +
            "Single Controller:\n" + 

            // Driver controls.
            "- Sticks: Field oriented controls\n" +
            "- Right Bumper: Shift Up\n" +
            "- Left Bumper: Shift Down\n"  +
            "- BACK (left small btn): Reset Gyro\n" +
            "- DPAD: Fine Alignment\n" +

            // Operator normal buttons
            "- X (press): Roller Down\n" +
            "- X (hold): Roller Spin\n" + 
            "- B (hold): Roller Expel + Arm Stop\n" + 
            "- Y : Roller Off + Arm Up \n" +
            
            // Operator override buttons
            "- LT (hold): Switch over to OP override mode\n" +
            "- LS Left+Right Override: Manually move intake\n" +
            "- RS Up+Down Override: Manually move climber";

        SmartDashboard.putString("Controller Binds", controllerInstructions);

        // Driver controls
        new JoystickButton(getDeadbandedDriverController(), XboxController.BACK_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.resetGyro()));

        new JoystickButton(getDeadbandedDriverController(), XboxController.RIGHT_BUMPER_BUTTON) // final
            .onTrue(new InstantCommand(()  -> m_robotSwerveDrive.shiftUp()));

        new JoystickButton(getDeadbandedDriverController(), XboxController.LEFT_BUMPER_BUTTON) // final
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.shiftDown()));

        // Fine Alignment
        new Trigger(() -> getDeadbandedDriverController().getPOV() != -1)
            .whileTrue(new RunCommand(
                () -> m_robotSwerveDrive.driveFine(
                    new Translation2d(
                        1, 
                        Rotation2d.fromDegrees(getDeadbandedDriverController().getPOV())
                    ), 
                    getDeadbandedDriverController().getRight(), 0.15
                ), m_robotSwerveDrive))
            .onFalse(new InstantCommand(() -> m_robotSwerveDrive.softStop(), m_robotSwerveDrive));
        
        

        
        // Operator controls (Non-override, LT UP)


        // Arm down
        new Trigger(() -> !lt_down() && getDeadbandedDriverController().getXButton())
            .onTrue(new InstantCommand(() -> {
                m_robotMap.m_robotIntake.PIDOut();
                m_robotMap.m_robotIntake.spinIntakeMotor();
            }, m_robotMap.m_robotIntake))
            .onFalse(new InstantCommand(() -> m_robotMap.m_robotIntake.stopArmMotor(), m_robotMap.m_robotIntake));
        
        // Arm up
        new Trigger(() -> !lt_down() && getDeadbandedDriverController().getYButton())
            .onTrue(new InstantCommand(() -> {
                m_robotMap.m_robotIntake.PIDIn();
                m_robotMap.m_robotIntake.stopIntakeMotors();
            }, m_robotMap.m_robotIntake))
            .onFalse(new InstantCommand(() -> m_robotMap.m_robotIntake.stopArmMotor(), m_robotMap.m_robotIntake));
        
        // Handoff / spit out
        new Trigger(() -> !lt_down() && getDeadbandedDriverController().getAButton())
            .onTrue(new InstantCommand(() -> {
                // m_robotMap.m_robotIntake.PIDIn();
                m_robotMap.m_robotIntake.handoff();
            }, m_robotMap.m_robotIntake))
            .onFalse(new InstantCommand(() -> m_robotMap.m_robotIntake.stopIntakeMotors(), m_robotMap.m_robotIntake));

        // Shoot
        new Trigger(() -> !lt_down() && rt_down())
            .onTrue(new InstantCommand(() -> m_robotMap.m_robotShooter.spin(0.5), m_robotMap.m_robotShooter))
            .onFalse(new InstantCommand(() -> m_robotMap.m_robotShooter.stop(), m_robotMap.m_robotShooter));
            


        // OP Override
        

    }

//.onTrue(new InstantCommand(()  -> m_robotLED.setMode(LEDPatterns.SOLID_PINK_HOT)));

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {


        //return autoPlayback;
        //return new GotoPositionCommand(m_robotSwerveDrive, m_vision)
        //return autoChooser.getSelected();
    // try{
    // //     // Load the path you want to follow using its name in the GUI
    //     return autoCommand;
    // } catch (Exception e) {
    //     DriverStation.reportError("Path planner error: " + e.getMessage(), e.getStackTrace());
        return autoCommand;
    // }
    // return new PathPlannerAuto("Line-up-no-arm");
    // zach told me to do the below comment
    //return new GotoPositionCommand(m_robotSwerveDrive, m_vision);
      //  return new GotoPositionCommand(m_robotSwerveDrive, m_vision, AutoConstants.targetpos);
    }

    public boolean autoChooserUpdated = false;
    public void makeAutoChooser() {
        autoChooser = new SendableChooser<String>();
        autoChooser.setDefaultOption("None", "None");
        File dir;

        if(RobotBase.isReal()) {
            dir = new File("/home/lvuser/deploy/pathplanner/autos/");
        } else {
            // dir = new File("C:\\Users\\Ridgebotics\\Documents\\GitHub\\2025RidgeScape\\src\\main\\deploy\\pathplanner\\autos\\");
            dir = new File("C:\\Users\\Ridgebotics\\Documents\\GitHub\\2026KPopRobotHunters\\src\\main\\deploy\\pathplanner\\autos\\");
        }

        String[] autos = dir.list();

        if(autos == null) return;

        for (String auto : autos) {
            if (auto.endsWith(".auto"))
                autoChooser.addOption(auto.replaceAll(".auto", ""), auto.replaceAll(".auto", ""));
            // System.out.println(auto);
        }

        autoChooser.onChange((filename) -> {
            autoChooserUpdated = true;
            if (filename == null || filename.equals("None")) {
                autoCommand = null;
                return;
            }
            // if (filename.equals("Taxi%")) {
            //     autoCommand = new SequentialCommandGroup(
            //         new MoveForTimeCommand(m_robotSwerveDrive, 
            //             new Translation2d(0, -1), 
            //             new Translation2d(), 1000, true
            //     ), new InstantCommand(()-> {m_robotSwerveDrive.softStop();} , m_robotSwerveDrive));
            // } else {
                autoCommand = new PathPlannerAuto(filename);
            // }
            System.out.println("Robot Auto Changed " + filename);

            //----
            PathPlannerAuto auto = new PathPlannerAuto(filename);
            m_robotSwerveDrive.setInitalPose(auto.getStartingPose());
            //-----
        });
        SmartDashboard.putData(autoChooser);

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

    // public ButtonBox getButtonBox() {
    //     return this.m_buttonBox;
    // }
}