// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands.Autos;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc4388.robot.commands.PID;
import frc4388.robot.subsystems.Intake;
import frc4388.utility.RobotGyro;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalance extends PID {
  RobotGyro gyro;
  Intake intake;
  /** Creates a new AutoBalance. */
  public AutoBalance(RobotGyro gyro, Intake intake) {
    super(0.6, 0, 0, 0, 0);

    this.gyro = gyro;
    this.intake = intake;

    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(intake);
  }

  // Returns true when the command should end.

  public double getError() {
		var pitch = gyro.getRoll();
		SmartDashboard.putNumber("pitch", pitch);
		return pitch;
	}

	@Override
	public void runWithOutput(double output) {
		double out2 = MathUtil.clamp(output / 40, -59, 0);
		if (Math.abs(getError()) < 3) out2 = 0;
    intake.talonPIDPosition(out2);
  }

}
