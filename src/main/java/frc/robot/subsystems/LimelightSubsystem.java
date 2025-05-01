package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightSubsystem extends SubsystemBase {
    private final NetworkTableEntry txnc;
    NetworkTable llight;
    private final NetworkTableEntry tx;
    private final NetworkTableEntry ty;
    private final NetworkTableEntry ta;
    private final NetworkTableEntry tv;

    public LimelightSubsystem(NetworkTable limeLight) {
        llight = limeLight;
        ta = llight.getEntry("ta"); //area of reflective object
        tx = llight.getEntry("tx"); //displacement on x axis
        ty = llight.getEntry("ty"); //displacement on y axis
        tv = llight.getEntry("tv"); //0 or 1 depending on if there is a reflective object
        txnc = llight.getEntry("txnc"); // Angle on the april tag
    }

    public void printDebug() {
        System.out.println(txnc.getDouble(0.0));
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
        double kP = .001; // .035;

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
        double kP = 0.1;
        double maxArea = 1.8;
        double ta = this.ta.getDouble(0.0);
        // TODO: Make the parabola more flat
        if (ta == 0) return 0;
        double t = ta - maxArea;
        double targetingForwardSpeed = Math.pow(t, 2) * kP;
        targetingForwardSpeed = Math.max(
                targetingForwardSpeed,
                Constants.DriveConstants.kMaxSpeedMetersPerSecond
        );
        targetingForwardSpeed *= t < 0 ? -1 : 1;

        System.out.println(targetingForwardSpeed);
        return targetingForwardSpeed;
    }

    private double limelightStrafeProportional(double target) {
        final double aggressiveness = 0.01;
        return (tx.getDouble(0.0) + target)
                * aggressiveness;
    }

    public void autonomous(DriveSubsystem driveSub, double target) {
        final double sidewaysMove = limelightStrafeProportional(target);
        driveSub.drive(0, sidewaysMove, 0, false);
    }

    public void driveOld(DriveSubsystem robotDrive) {
        var area = ta.getDouble(0.0);
        var x = tx.getDouble(0.0);
        var y = ty.getDouble(0.0);
        var isSomething = tv.getDouble(0.0);
        //CHECK IF SOMETHING'S HERE THEN TURN TOWARD IT
        if (isSomething == 1) {
            final double dist = 1f;
            final double rotationSpeed = -(x * (1.0 / 40)) * 0.3;
            if (area < dist) {
                final double forwardSpeed = (Math.sqrt(3) - Math.sqrt(area)) / 2;
                robotDrive.drive(-forwardSpeed, 0, rotationSpeed, false);
            } else if (area >= dist) {
                final double backwardSpeed = (1 - Math.cbrt(area));
                robotDrive.drive(-backwardSpeed, 0, rotationSpeed, false);
            }
        } else {
            robotDrive.drive(0, 0, 0, false);
        }
    }

    public void drive(double rawAxis, DriveSubsystem robotDrive) {
        final double rot_limelight = limelightAimProportional();
        final double forward_limelight = limelightRangeProportional();

        robotDrive.drive(forward_limelight, rawAxis, rot_limelight, false);
    }

}
