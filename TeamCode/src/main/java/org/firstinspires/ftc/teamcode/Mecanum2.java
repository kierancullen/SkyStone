package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    Servo turn;
    Servo arm1, arm2, arm3, arm4;
    Servo grip;


    GrabLiftPlaceController g;

    @Override
    public void init() {

        super.init();
         winchLeft = hardwareMap.get(DcMotor.class, "winchLeft");
         winchLeft.setDirection(DcMotorSimple.Direction.REVERSE);
         winchRight = hardwareMap.get(DcMotor.class,"winchRight");

         winchLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         winchRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         swing = hardwareMap.get(Servo.class, "swingLeft");
         swing.setDirection(Servo.Direction.REVERSE);
         grip = hardwareMap.get(Servo.class, "grip");
         tilt = hardwareMap.get(Servo.class, "swingRight");
         arm1 = hardwareMap.get(Servo.class, "arm1");
         arm2 = hardwareMap.get(Servo.class, "arm2");
         arm1.setDirection(Servo.Direction.REVERSE);
        setServoExtendedRange(arm1, 500, 2500);
        setServoExtendedRange(arm2, 500, 2500);
         arm1.setPosition(0.5);
         arm2.setPosition(0.5);
         grip.setPosition(0);


         //grab = hardwareMap.get(Servo.class, "grab");
         //turn = hardwareMap.get(Servo.class, "turn");



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
        winchLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        winchRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    int servoIndex = 0;
    int motorIndex = 0;

    double pwm = 0.33;
    double motorPower = 0.2;
    int targetPos = 0;

    double gripPos = 0;
    @Override
    public void loop() {
        /*GlobalMovement.updateFromGamepad(gamepad1);
        telemetry.addData("Left:", winchLeft.getCurrentPosition());
        telemetry.addData("Right:", winchRight.getCurrentPosition());

        //g.tick();
        */
        /*Servo[] servos = {arm1, arm2};
        if (gamepad1.b) servoIndex++;
        if (servoIndex > 1) servoIndex=0;
        if (gamepad1.y) pwm+= 0.002;
        if (gamepad1.x) pwm-= 0.002;

        if (pwm>1) pwm=1;
        if (pwm<0) pwm =0;

        servos[servoIndex].setPosition(pwm); */

        if (gamepad1.y) pwm +=0.002;
        if (gamepad1.x) pwm -= 0.002;
        if (pwm>1) pwm=1;
        if (pwm<0) pwm =0;

        arm1.setPosition(pwm);
        arm2.setPosition(pwm);



        if (gamepad1.right_bumper) {
            gripPos += 0.02;
        }
        if (gamepad1.left_bumper) {
            gripPos -= 0.02;
        }

        grip.setPosition(gripPos);




        telemetry.addData("Servo:", servoIndex);
        telemetry.addData("Servo Pos:", pwm);
        telemetry.addData("wl:", winchLeft.getCurrentPosition());
        telemetry.addData("wr:", winchRight.getCurrentPosition());
        telemetry.update();

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
