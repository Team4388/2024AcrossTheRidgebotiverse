package frc4388.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {
    private final NetworkTableEntry nt_ll_tx;
    private final NetworkTableEntry nt_ll_ty;
    private final NetworkTableEntry nt_ll_ta;
    private final NetworkTableEntry nt_ll_ts;

    public Vision() {
        final var tagTable = NetworkTableInstance.getDefault().getTable("limelight`");

        nt_ll_tx = tagTable.getEntry("tx"); //TODO
        nt_ll_ty = tagTable.getEntry("ty"); //TODO
        nt_ll_ta = tagTable.getEntry("ta"); //TODO
        nt_ll_ts = tagTable.getEntry("ts"); //TODO

        // I do not know what these values represent
        // There are diffrent values that are sent
        // when "3D" mode is enabled on the limelight
        // So this needs to be updated!
    }

    //    public AprilTag[] getAprilTags() { This func should return an apriltag object!
    public void getAprilTags() {
        // if (!m_isTags.getBoolean(false)) return new AprilTag[0];

        double ll_tx = nt_ll_tx.getDouble(new Double(0));
        double ll_ty = nt_ll_ty.getDouble(new Double(0));
        double ll_ta = nt_ll_ta.getDouble(new Double(0));
        double ll_ts = nt_ll_ts.getDouble(new Double(0));

        // AprilTag tags[] = new AprilTag[xarr.length];
        // for (int i = 0; i < tags.length; i++) {
        //     tags[i] = new AprilTag(0, new Pose3d(xarr[i], yarr[i], zarr[i], new Rotation3d()));
        // }

        // return tags;

        
    }
}
