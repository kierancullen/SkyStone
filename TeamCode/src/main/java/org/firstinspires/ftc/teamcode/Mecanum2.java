package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
    Servo gate;

    AnalogInput sonarLeft;
    DigitalChannel triggerLeft;
    AnalogInput sonarBackLeft;
    DigitalChannel triggerBackLeft;
    AnalogInput sonarBackRight;
    DigitalChannel triggerBackRight;

    DistanceSensor ramp;
    DistanceSensor floor;


    GrabLiftPlaceController g;

    @Override
    public void init() {

        super.init();
         winchLeft = hardwareMap.get(DcMotor.class, "winchLeft");
         winchLeft.setDirection(DcMotorSimple.Direction.REVERSE);
         winchRight = hardwareMap.get(DcMotor.class,"winchRight");

         winchLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         winchRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         winchLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        winchRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

         swing = hardwareMap.get(Servo.class, "swingLeft");
         swing.setDirection(Servo.Direction.REVERSE);
         grip = hardwareMap.get(Servo.class, "grip");
         grip.setDirection(Servo.Direction.REVERSE);
         tilt = hardwareMap.get(Servo.class, "swingRight");
         arm1 = hardwareMap.get(Servo.class, "arm1");
         arm2 = hardwareMap.get(Servo.class, "arm2");
         arm2.setDirection(Servo.Direction.REVERSE);
        setServoExtendedRange(arm1, 500, 2500);
        setServoExtendedRange(arm2, 500, 2500);
         arm1.setPosition(0.5);
         arm2.setPosition(0.5);
         grip.setPosition(0);
         sonarLeft = hardwareMap.analogInput.get("sonarLeft");
         triggerLeft = hardwareMap.digitalChannel.get("triggerLeft");
         triggerLeft.setMode(DigitalChannel.Mode.OUTPUT);

         gate = hardwareMap.get(Servo.class, "gate");

        sonarBackLeft = hardwareMap.analogInput.get("sonarBackLeft");
        triggerBackLeft = hardwareMap.digitalChannel.get("triggerBackLeft");
        triggerBackLeft.setMode(DigitalChannel.Mode.OUTPUT);

        sonarBackRight = hardwareMap.analogInput.get("sonarBackRight");
        triggerBackRight= hardwareMap.digitalChannel.get("triggerBackRight");
        triggerBackRight.setMode(DigitalChannel.Mode.OUTPUT);

        ramp = hardwareMap.get(DistanceSensor.class, "ramp");
        floor = hardwareMap.get(DistanceSensor.class, "floor");

        triggerLeft.setState(false);
        triggerBackLeft.setState(false);
        triggerBackRight.setState(false);


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

    double pwm = 0.5;
    double motorPower = 0.2;
    int targetPos = 0;

    double gripPos = 0;
    boolean rampDetected;
    boolean floorDetected;


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

        if (gamepad1.y) pwm+= 0.02;
        if (gamepad1.x) pwm-= 0.02;
        if (pwm>1) pwm=1;
        if (pwm<0) pwm =0;

        arm1.setPosition(pwm);
        arm2.setPosition(pwm);





        if (gamepad1.right_bumper) {
            gripPos += 0.06;
        }
        if (gamepad1.left_bumper) {
            gripPos -= 0.06;
        }

        grip.setPosition(gripPos);

        if (gamepad1.a) {
            triggerLeft.setState(true);
            //triggerBackLeft.setState(true);
            triggerBackRight.setState(true);

        }


            if (gamepad1.dpad_up) {
                if (winchLeft.getCurrentPosition() > 1150) {
                    winchRight.setPower(0);
                    winchLeft.setPower(0);
                }
                winchRight.setPower(1.0);
                winchLeft.setPower(1.0);
            }
            else if (gamepad1.dpad_down) {
                if (winchLeft.getCurrentPosition() < 10) {
                    winchRight.setPower(0);
                    winchLeft.setPower(0);
                }
                else {
                    winchRight.setPower(-0.5);
                    winchLeft.setPower(-0.5);
                }

            }
            else {
                winchRight.setPower(0);
                winchLeft.setPower(0);
            }



        triggerLeft.setState(false);
        triggerBackLeft.setState(false);
        triggerBackRight.setState(false);

        if (gamepad2.back) {
            triggerLeft.setState(true);
            triggerBackLeft.setState(true);
            triggerBackRight.setState(true);
            triggerLeft.setState(false);
            triggerBackLeft.setState(false);
            triggerBackRight.setState(false);
        }

        rampDetected = ramp.getDistance(DistanceUnit.MM) < 100;
        floorDetected = floor.getDistance(DistanceUnit.MM) < 100;

        telemetry.addData("Lift: ", (winchLeft.getCurrentPosition() + winchRight.getCurrentPosition()) / 2);
        telemetry.addData("Servo:", servoIndex);
        telemetry.addData("Servo Pos:", pwm);
        telemetry.addData("Grip Pos:", gripPos);
        telemetry.addData("wl:", winchLeft.getCurrentPosition());
        telemetry.addData("wr:", winchRight.getCurrentPosition());
        telemetry.addData("sonarLeft (in):", 73.123*sonarLeft.getVoltage());
        telemetry.addData("sonarBackLeft (in):", 73.123*sonarBackLeft.getVoltage());
        telemetry.addData("sonarBackRight (in):", 73.123*sonarBackRight.getVoltage());
        //telemetry.addData("floor (mm):", floor.getDistance(DistanceUnit.MM));
        //telemetry.addData("ramp (mm):", ramp.getDistance(DistanceUnit.MM));

        telemetry.addData("ramp:", rampDetected);
        telemetry.addData("floor:", floorDetected);


        //(5*(sonarLeft.getVoltage()/(3.3/1024)))/25.4
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
