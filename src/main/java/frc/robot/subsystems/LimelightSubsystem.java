package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightSubsystem extends SubsystemBase {
    NetworkTable llight;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;
    private NetworkTableEntry tv;
    private double x;
    private double y;
    private double area;
    private Long isSomething;

    public LimelightSubsystem(NetworkTable limeLight) {
        llight = limeLight;
        ta = llight.getEntry("ta"); //area of reflective object
        tx = llight.getEntry("tx"); //displacement on x axis
        ty = llight.getEntry("ty"); //displacement on y axis
        tv = llight.getEntry("tv"); //0 or 1 depending on if there is a reflective object
    }

    // simple proportional turning control with Limelight.
    // "proportional control" is a control algorithm in which the output is proportional to the error.
    // in this case, we are going to return an angular velocity that is proportional to the
    // "tx" value from the Limelight.
    double limelightAimProportional() {
        // kP (constant of proportionality)
        // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
        // if it is too high, the robot will oscillate around.
        // if it is too low, the robot will never reach its target
        // if the robot never turns in the correct direction, kP should be inverted.
        double kP = .005; // .035;

        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of
        // your limelight 3 feed, tx should return roughly 31 degrees.
        double targetingAngularVelocity = tx.getDouble(0.0) * kP;

        // convert to radians per second for our drive method
        targetingAngularVelocity *= Constants.DriveConstants.kMaxAngularSpeed;

        //invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1.0;

        return targetingAngularVelocity;
    }

    // simple proportional ranging control with Limelight's "ty" value
    // this works best if your Limelight's mount height and target mount height are different.
    // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
    double limelightRangeProportional() {
        double kP = .1;
        double targetingForwardSpeed = ty.getDouble(0.0) * kP;
        targetingForwardSpeed *= Constants.DriveConstants.kMaxSpeedMetersPerSecond;
        targetingForwardSpeed *= -1.0;
        return targetingForwardSpeed;
    }


    public void drive(double rawAxis, DriveSubsystem robotDrive) {
        final double rot_limelight = limelightAimProportional();
        final double forward_limelight = limelightRangeProportional();

        robotDrive.drive(forward_limelight, rawAxis, rot_limelight, false);
    }

}
