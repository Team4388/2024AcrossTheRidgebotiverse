// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc4388.robot.Constants.VisionConstants;

// Look at vvv for networktables stuff
// https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api#apriltag-and-3d-data

public class Limelight extends SubsystemBase {

  // // [X, Y, Z, Roll, Pitch, Yaw]
  // private double[] cameraPose;
  // private boolean isTag;

  // private Pose2d pose;
  // private boolean isNearSpeaker;

  // public boolean getIsTag() {
  //   return isTag;
  // }

  // private void update() {
  //   SmartDashboard.putBoolean("Apriltag", isTag);
  //   if(!isTag){
  //     return;
  //   }

  //   double x = cameraPose[0];
  //   double y = cameraPose[1];
  //   double yaw = cameraPose[5];

  //   Rotation2d rot = Rotation2d.fromDegrees(yaw);

  //   pose = new Pose2d(x, y, rot);

  //   boolean isRed = DriverStation.getAlliance().get() == Alliance.Red;
    
  //   double distance;

  //   if(isRed){
  //     distance = pose.getTranslation().getDistance(VisionConstants.RedSpeakerCenter);
  //   }else{
  //     distance = pose.getTranslation().getDistance(VisionConstants.BlueSpeakerCenter);
  //   }
    
  //   isNearSpeaker = distance <= VisionConstants.SpeakerBubbleDistance;

  //   //SmartDashboard.putBoolean("nearSpeaker", isNearSpeaker);
  //   //SmartDashboard.putNumber("speakerDistance", distance);
  // }

  // public Pose2d getPose() {
  //   return pose;
  // }

  // public boolean isNearSpeaker() {
  //   return isNearSpeaker;
  // }

  // @Override
  // public void periodic() {
  //   // This method will be called once per scheduler run

  //   //isTag = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0) == 1.0;
  //   //double[] newPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);

  //   //if(newPose != cameraPose){
  //   //  cameraPose = newPose;
  //     //update();
  //   //}
  // }
}