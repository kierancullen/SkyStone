package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="OdometryTests")
public class OdometryTests extends RobotOpModeSwing {

    DcMotor yLeft;
    DcMotor yRight;
    DcMotor x;

    public void init() {
        super.init();
        yLeft = myDrivetrain.getYOdometerLeft();
        yRight = myDrivetrain.getYOdometerRight();
        x = myDrivetrain.getXOdometer();
    }

    public void loop() {
        telemetry.addData("yLeft:",  yLeft.getCurrentPosition());
        telemetry.addData("yRight:", yRight.getCurrentPosition());
        telemetry.addData("x:", x.getCurrentPosition());
        super.loop();
    }
}
