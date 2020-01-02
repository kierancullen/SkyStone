package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="Mecanum2", group="Mecanum")
public class Mecanum2 extends RobotOpMode {

    DcMotor winchLeft;
    DcMotor winchRight;
    Servo swing;
    Servo tilt;
    Servo grab;
    GrabLiftPlaceController g;

    @Override
    public void init() {

        super.init();
         winchLeft = hardwareMap.get(DcMotor.class, "winchLeft");
         winchRight = hardwareMap.get(DcMotor.class,"winchRight");

         winchLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         winchRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         swing = hardwareMap.get(Servo.class, "swingLeft");
         swing.setDirection(Servo.Direction.REVERSE);
         tilt = hardwareMap.get(Servo.class, "swingRight");
         grab = hardwareMap.get(Servo.class, "grab");

        setServoExtendedRange(swing, 500, 2500);
        setServoExtendedRange(tilt, 500, 2500);
        setServoExtendedRange(grab, 500, 2500);

        //g = new GrabLiftPlaceController(winchLeft, winchRight, swing, tilt, grab);

    }

    public void setServoExtendedRange(Servo servo, int min, int max) {
        ServoControllerEx controller = (ServoControllerEx) servo.getController();
        int servoPort = servo.getPortNumber();
        PwmControl.PwmRange range = new PwmControl.PwmRange(min, max);
        controller.setServoPwmRange(servoPort, range);
    }

    public void start() {
        //g.start();
    }

    int servoIndex = 0;
    int motorIndex = 0;

    double pwm = 0;
    double motorPower = 0.2;
    int targetPos = 0;

    @Override
    public void loop() {
        /*GlobalMovement.updateFromGamepad(gamepad1);
        telemetry.addData("Left:", winchLeft.getCurrentPosition());
        telemetry.addData("Right:", winchRight.getCurrentPosition());

        //g.tick();
        */
       Servo[] servos = {swing, tilt, grab};
        if (gamepad1.b) servoIndex++;
        if (servoIndex > 2) servoIndex=0;
        if (gamepad1.y) pwm+= 0.002;
        if (gamepad1.x) pwm-= 0.002;

        if (pwm>1) pwm=1;
        if (pwm<0) pwm =0;

        servos[servoIndex].setPosition(pwm);
        telemetry.addData("Servo:", servoIndex);
        telemetry.addData("Servo Pos:", pwm);

        /*
        DcMotor[] motors = {winchLeft, winchRight};
        if (gamepad1.right_bumper) {
            motors[motorIndex].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorIndex++;
            if (motorIndex > 1) motorIndex=0;
            motors[motorIndex].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }



        motors[motorIndex].setPower(motorPower);
        if (gamepad1.dpad_up) targetPos++;
        if (gamepad1.dpad_down) targetPos--;

        motors[motorIndex].setTargetPosition(targetPos);

        telemetry.addData("Motor:", motorIndex);
        telemetry.addData("Target Pos:", targetPos);

        super.loop(); */

    }
}
