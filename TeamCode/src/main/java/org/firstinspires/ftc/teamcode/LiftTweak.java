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

    public void loop() {
        winchLeft.setPower(-gamepad1.right_stick_y);
        winchRight.setPower(-gamepad1.right_stick_y);
    }
}
