package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="Odometry")

public class RobotOpModeTest extends OpMode {

    Drivetrain myDrivetrain;
    BNO055IMU imu;
    AutopilotTrackerDualOdo tracker;

    public void init() {


        DcMotor tl = hardwareMap.get(DcMotor.class, "tl");
        DcMotor tr = hardwareMap.get(DcMotor.class, "tr");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");

        tl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        myDrivetrain = new Drivetrain (tl, tr, bl, br);

        //The OFS is plugged into motor encoder ports, so we need to do this:
        GlobalPosition.OFS_x = br;
        GlobalPosition.OFS_y = bl;


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "IMU");
        imu.initialize(parameters);

        GlobalPosition.imu = imu;

        tracker = new AutopilotTrackerDualOdo(br, bl, -4.3, -4.3, new double[]{0, 0, 0}, 1440/(1.89*Math.PI), imu);
        tracker.setInverts(false, true);
    }

    public void loop() {

        myDrivetrain.updatePowers();
        GlobalPosition.updatePosition();
        GlobalPosition.sendDebugPosition();

        tracker.update();
        double[] robotPosition = tracker.getRobotPosition();

        telemetry.addData("x", robotPosition[0]);
        telemetry.addData("y", robotPosition[1]);
        telemetry.update();


    }


}
