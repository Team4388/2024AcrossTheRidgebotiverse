// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Look at vvv for networktables stuff
// https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api#apriltag-and-3d-data

public class Limelight extends SubsystemBase {
  private double[] cameraPose;
  private Boolean isTag;

  public boolean getIsTag() {
    return isTag;
  }

  public Pose2d getPose() {
    //TODO - Get actual values!

    double x = 0;
    double y = 0;
    double yaw = 0;

    Rotation2d rot = Rotation2d.fromDegrees(yaw);

    return new Pose2d(x, y, rot);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    isTag = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getBoolean(false);
    cameraPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
  }
}