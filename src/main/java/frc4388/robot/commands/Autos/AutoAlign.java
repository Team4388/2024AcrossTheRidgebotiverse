package frc4388.robot.commands.Autos;
import frc4388.robot.subsystems.Limelight;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.robot.Constants.AutoAlignConstants;
import frc4388.robot.Constants.VisionConstants;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.Optional;

import org.opencv.core.RotatedRect;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AutoAlign extends Command {
    private SwerveDrive swerve;
    private Limelight limelight;
    private Pose2d pose;
    private Translation2d targetPos;
    private Rotation2d targetRot;

    private Optional<Alliance> alliance;
    private boolean isRed;

    private boolean isFinished;
    private boolean isReverseFinished;

    private boolean reverseAfterFinish;
    private Translation2d[] moveStickReplayArr;
    private Translation2d[] rotStickReplayArr;
    private int replayIndex;

    // PID Stuff
    private double prevError;
    private double cumError;

    public AutoAlign(SwerveDrive swerve, Limelight limelight) {
        this.swerve = swerve;
        this.limelight = limelight;
    }

    // Calc the closest point on a circle, to the center of the speaker 
    private Translation2d calcTargetPos(){
        final double R = VisionConstants.targetPosDistance;
        final double cX;
        final double cY;
        if(isRed){
            cX = VisionConstants.RedSpeakerCenter.getX();
            cY = VisionConstants.RedSpeakerCenter.getY();
        }else{
            cX = VisionConstants.BlueSpeakerCenter.getX();
            cY = VisionConstants.BlueSpeakerCenter.getY();
        }
        final double pX = pose.getX();
        final double pY = pose.getY();

        // Code is from https://stackoverflow.com/questions/300871/best-way-to-find-a-point-on-a-circle-closest-to-a-given-point
        double vX = pX - cX;
        double vY = pY - cY;
        double magV = Math.sqrt(vX*vX + vY*vY);
        double aX = cX + vX / magV * R;
        double aY = cY + vY / magV * R;

        return new Translation2d(aX, aY);
    }

    // Calculate the angle facing the center, at the target rot
    private Rotation2d calcTargetRot() {
        final double R = VisionConstants.targetPosDistance;
        final double cX;
        final double cY;
        if(isRed){
            cX = VisionConstants.RedSpeakerCenter.getX();
            cY = VisionConstants.RedSpeakerCenter.getY();
        }else{
            cX = VisionConstants.BlueSpeakerCenter.getX();
            cY = VisionConstants.BlueSpeakerCenter.getY();
        }
        final double pX = pose.getX();
        final double pY = pose.getY();

        final double dX = pX - cX;
        final double dY = pY - cY;

        final double yaw = ((Math.atan2(dX, dY)*360/Math.PI) % 360);

        return Rotation2d.fromDegrees(yaw);
    }

    // Clamp to a circle, like an xbox controller
    private Translation2d clamp(double oldX, double oldY){
        // Code is from https://stackoverflow.com/questions/74329985/how-can-i-clamp-a-position-to-a-circley
        final double alpha = (Math.atan2(oldX, -oldY) * 180 / Math.PI + 360) % 360;
        final double maxX = Math.abs(Math.cos(alpha / 180 * Math.PI));
        final double maxY = Math.abs(Math.sin(alpha / 180 * Math.PI));

        final double newX = Math.max(-maxX, Math.min(maxX, oldX));
        final double newY = Math.max(-maxY, Math.min(maxY, oldY));

        return new Translation2d(newX, newY);
    }

    private Translation2d calcMoveStick(){
        final double curX = pose.getX();
        final double curY = pose.getY();

        // I think this might work, assuming the constants are good
        double stickX = -(curX - targetPos.getX()) * AutoAlignConstants.MoveSpeed;
        double stickY = -(curY - targetPos.getY()) * AutoAlignConstants.MoveSpeed;

        return clamp(stickX, stickY);
    }

    private Translation2d calcRotStick(){
        double error = pose.getRotation().getDegrees() - targetRot.getDegrees();
	cumError += error * .02; // 20 ms
	double delta = error - prevError;

        final double kP = 4;
        final double kI = 4;
        final double kD = 4;
        final double kF = 4;

	double output = error * kP;
	output += cumError * kI;
	output += delta * kD;
	output += kF;

        prevError = error;
        return clamp(output, 0);
    }

    public void reverse() {
        this.reverseAfterFinish = true;
    }

    // Called when the command is initially scheduled.
    @Override 
    public final void initialize() {
        isRed = alliance.get() == DriverStation.Alliance.Red;
        if(reverseAfterFinish){
            // isReverseFinished = false;
            replayIndex = 0;
        }else{
            moveStickReplayArr = new Translation2d[]{};
            rotStickReplayArr = new Translation2d[]{};
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Update limelight pos
        pose = limelight.getPose();

        // These must be 0, or it will error
        Translation2d moveStick = new Translation2d(0, 0);
        Translation2d rotStick = new Translation2d(0, 0);

        // Regular replay
        if(!isFinished){
            targetPos = calcTargetPos();
            targetRot = calcTargetRot();

            moveStick = calcMoveStick();
            rotStick = calcRotStick();

            // This is a way of appending...
            moveStickReplayArr[moveStickReplayArr.length] = moveStick;
            rotStickReplayArr[rotStickReplayArr.length] = rotStick;

            // if(isFinished != limelight.isNearSpeaker() && isReverseFinished){
            //     replayIndex
            // }
            isFinished = limelight.isNearSpeaker();

	    // If reverseAfterFinish, then loop back over and replay in reverse
        }else if(reverseAfterFinish && !isReverseFinished){
            // Get reverse direction
            moveStick = moveStickReplayArr[replayIndex-moveStickReplayArr.length-1];
            rotStick = rotStickReplayArr[replayIndex-rotStickReplayArr.length-1];

            // Invert sticks
            moveStick = new Translation2d(moveStick.getX()*-1, moveStick.getY()*-1);
            rotStick = new Translation2d(rotStick.getX()*-1, rotStick.getY()*-1);

            replayIndex++;

            if(replayIndex >= moveStickReplayArr.length){
                reverseAfterFinish = true;
            }
        }

        // This would greatly benifit from having feild Relative implemented.
        swerve.driveWithInput(moveStick, rotStick, true);
    }

    // Returns true when the command should end.
    @Override
    public final boolean isFinished() {
        return isFinished && (isReverseFinished || !reverseAfterFinish);
    }
}
