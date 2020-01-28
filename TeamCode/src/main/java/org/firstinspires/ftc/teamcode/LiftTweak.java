package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="LiftTweak")

public class LiftTweak extends RobotOpMode {

    DcMotor winchLeft;
    DcMotor winchRight;

    @Override
    public void init() {
        winchLeft = hardwareMap.get(DcMotor .class, "winchLeft");
        winchRight = hardwareMap.get(DcMotor.class,"winchRight");

        winchLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        winchLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        winchLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winchRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        winchRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winchLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        winchRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    int position;

    public void loop() {

        position = (winchLeft.getCurrentPosition() + winchRight.getCurrentPosition()) / 2;

        if (position > 900) {
            winchLeft.setPower(0);
            winchRight.setPower(0);
        }
        else if (gamepad1.dpad_up) {
            winchLeft.setPower(0.8);
            winchRight.setPower(0.8);
        }
        else if (gamepad1.dpad_down) {
            winchLeft.setPower(-0.3);
            winchRight.setPower(-0.3);
        }
        else {
            winchLeft.setPower(0);
            winchRight.setPower(0);
        }
    }
}
