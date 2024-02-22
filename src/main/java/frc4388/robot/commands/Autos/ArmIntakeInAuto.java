// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc4388.robot.commands.Intake.ArmIntakeIn;
import frc4388.robot.commands.Swerve.JoystickPlayback;
import frc4388.robot.subsystems.Intake;
import frc4388.robot.subsystems.Shooter;
import frc4388.robot.subsystems.SwerveDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmIntakeInAuto extends ParallelCommandGroup {
  private final Intake intake;
  private final Shooter shooter;
  private final SwerveDrive swerve;
  /** Creates a new ArmIntakeInAuto. */
  public ArmIntakeInAuto(Intake intake, Shooter shooter, SwerveDrive swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.intake = intake;
    this.shooter = shooter;
    this.swerve = swerve;
    addCommands(new ArmIntakeIn(intake, shooter), new WaitCommand(0.5).andThen(new JoystickPlayback(swerve, "TwoNotePrt1.txt")));
  }
}
