package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDriveREVOptimized;

@Autonomous(name = "AutoCommonRoadRunner")
public class AutoCommonRoadrunner extends LinearOpMode {

    SampleMecanumDriveREVOptimized drive;

    boolean invert = false;

    public void rrGoTo(Path path, double maxVel, double maxAccel, double maxAngVel, double maxAngAccel ) {
        //DriveConstraints contraints = new DriveConstraints(maxVel, maxAccel, 0, maxAngVel, maxAngAccel, 0;
        //TrajectoryGenerator gen = new TrajectoryGenerator();
        //Trajectory trajectory = TrajectoryGenerator.generateTrajectory(path, contraints);
    }

    public Pose2d getPose2d(double[] point, double heading) {
        double newX;
        double newY;
        double newHeading;

        if (!invert) {
            newX = point[0] - 72;
            newY = point[1] - 72;
            newHeading = heading + Math.toRadians(90);
        }
        else {
            newX = point[0] + 72;
            newY = point[1] - 72;
            newHeading = heading - Math.toRadians(90);
        }
    return new Pose2d(newX, newY, newHeading);
    }

    public Vector2d getVector2d(double[] point, double heading) {
        double newX;
        double newY;
        double newHeading;

        if (!invert) {
            newX = point[0] - 72;
            newY = point[1] - 72;
            newHeading = heading + Math.toRadians(90);
        }
        else {
            newX = point[0] + 72;
            newY = point[1] - 72;
            newHeading = heading - Math.toRadians(90);
        }
        return new Vector2d(newX, newY);
    }

    public double getHeading (double heading) {
        double newHeading;

        if (!invert) {
            newHeading = heading + Math.toRadians(90);
        }
        else {
            newHeading = heading - Math.toRadians(90);
        }

        return newHeading;
    }

    public void runOpMode() {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.setPoseEstimate(getPose2d(new double[] {36, 9.2, 0}, 0));
        waitForStart();
        if (isStopRequested()) return;
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                .splineTo(getPose2d(new double[] {26, 40, 0}, Math.toRadians(30)))
                .build()
        );


        DriveConstraints constraints = new DriveConstraints(50.0, 100.0, 0.0,
                5*Math.toRadians(180.0), 5*Math.toRadians(180.0), 0.0);
        TrajectoryBuilder builder = new TrajectoryBuilder(drive.getPoseEstimate(), constraints);
        builder.
                reverse()
                .splineTo(getPose2d(new double[] {36, 36, 0}, Math.toRadians(90)))
                .lineTo(getVector2d(new double[] {4*24, 36, 0}, Math.toRadians(90)));

        drive.followTrajectorySync(builder.build());





    }
}
