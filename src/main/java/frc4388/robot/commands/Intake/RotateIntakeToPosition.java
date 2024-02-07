// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc4388.robot.commands.Intake;


import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Translation2d;
import frc4388.robot.commands.PID;
import frc4388.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import frc4388.robot.subsystems.Intake;
public class RotateIntakeToPosition extends PID {

  Intake intake;
  double targetAngle;

  /** Creates a new PIDSparkMax. */
  public RotateIntakeToPosition(Intake intake, double targetAngle) {
    super(0.3, 0.0, 0.0, 0.0, 1);
    
    this.intake = intake;
    this.targetAngle = targetAngle;

    addRequirements(intake);
  }

  @Override
  public double getError() {
    return targetAngle - (((intake.getEncoder().getPosition()) * (360))%360);
  }

  @Override
  public void runWithOutput(double output) {
    intake.setVoltage(output / Math.abs(getError()));    
  }
}
