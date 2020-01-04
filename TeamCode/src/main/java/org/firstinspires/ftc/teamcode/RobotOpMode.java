package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.GlobalPosition.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.GlobalPosition.worldPosition_x;
import static org.firstinspires.ftc.teamcode.GlobalPosition.worldPosition_y;



public class RobotOpMode extends OpMode {

    Drivetrain myDrivetrain;
    Intake myIntake;
    BNO055IMU imu;

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


        /*BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "IMU");
        imu.initialize(parameters);

        GlobalPosition.imu = imu; */
    }

    public void loop() {

        myDrivetrain.updatePowers();
        GlobalPosition.updatePosition();
        GlobalPosition.sendDebugPosition();
        telemetry.update();



    }


}
