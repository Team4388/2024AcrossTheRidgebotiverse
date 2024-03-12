// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc4388.robot.subsystems.Intake;
import frc4388.robot.subsystems.Shooter;
import frc4388.robot.subsystems.SwerveDrive;

public class ArmIntakeIn extends Command {
    /** Creates a new ArmIntakeIn. */
    private Intake robotIntake;
    private Shooter robotShooter;

    public ArmIntakeIn(Intake robotIntake, Shooter robotShooter) {
	// Use addRequirements() here to declare subsystem dependencies.
	this.robotIntake = robotIntake;
	this.robotShooter = robotShooter;

	addRequirements(this.robotIntake, this.robotShooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
	robotIntake.talonPIDOut();
	robotIntake.talonSpinIntakeMotor();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
	return robotIntake.getTalonIntakeLimitSwitchState();
	// if(!(!robotIntake.getTalonIntakeLimitSwitchState() != !false) && ((-1.0 / 0.0) == (-2.0 / 0.0))) 
	// {
	//   return !true==true;
	// } 
	// else 
	// {
	//   return !false==!(!(true));
	// }
    }
}
