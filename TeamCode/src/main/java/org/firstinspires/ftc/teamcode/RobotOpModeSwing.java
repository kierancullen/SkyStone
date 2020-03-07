package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;import com.qualcomm.robotcore.eventloop.opmode.OpMode;


public class RobotOpModeSwing extends OpMode  {

    Drivetrain myDrivetrain;

    DcMotor winchLeft;
    DcMotor winchRight;

    DcMotor intakeLeft;
    DcMotor intakeRight;

    Servo arm1;
    Servo arm2;
    Servo swingLeft;
    Servo swingRight;

    Servo grip;
    Servo turn;

    Servo hook1;
    Servo hook2;


    Servo capstone;

    Servo gate;
    DistanceSensor ramp;
    DistanceSensor floor;


    IntakeController in;
    OuttakeController2 out;
    AnalogInput scotty;




    public void init() {


        winchLeft = hardwareMap.get(DcMotor.class, "winchLeft");
        winchLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        winchRight = hardwareMap.get(DcMotor.class,"winchRight");

        winchLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winchRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotor.class, "intakeRight");
        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");
        arm2.setDirection(Servo.Direction.REVERSE);


        setServoExtendedRange(arm1, 500, 2500);
        setServoExtendedRange(arm2, 500, 2500);

        swingLeft = hardwareMap.get(Servo.class, "swingLeft");
        swingRight = hardwareMap.get(Servo.class, "swingRight");
        swingRight.setDirection(Servo.Direction.REVERSE);




        setServoExtendedRange(swingLeft, 500, 2500);
        setServoExtendedRange(swingRight, 500, 2500);

        grip = hardwareMap.get(Servo.class, "grip");
        grip.setDirection(Servo.Direction.REVERSE);
        setServoExtendedRange(grip, 500, 2500);

        hook1 = hardwareMap.get(Servo.class, "hook1");
        hook1.setDirection(Servo.Direction.REVERSE);
        hook2 = hardwareMap.get(Servo.class, "hook2");
        hook1.setPosition(0.0);
        hook2.setPosition(0.0);

        gate = hardwareMap.get(Servo.class, "gate");
        ramp = hardwareMap.get(DistanceSensor.class, "ramp");
        floor = hardwareMap.get(DistanceSensor.class, "floor");

        capstone = hardwareMap.get(Servo.class, "capstone");
        capstone.setPosition(0);

        DcMotor tl = hardwareMap.get(DcMotor.class, "tl");
        DcMotor tr = hardwareMap.get(DcMotor.class, "tr");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");

        tl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        myDrivetrain = new Drivetrain (tl, tr, bl, br);

        scotty = hardwareMap.get(AnalogInput.class, "scotty");

        out = new OuttakeController2(winchRight, winchLeft, arm1, arm2, grip);
        in = new IntakeController (intakeLeft, intakeRight, swingLeft, swingRight, gate, out, ramp, floor);


    }

    public void setServoExtendedRange(Servo servo, int min, int max) {
        ServoControllerEx controller = (ServoControllerEx) servo.getController();
        int servoPort = servo.getPortNumber();
        PwmControl.PwmRange range = new PwmControl.PwmRange(min, max);
        controller.setServoPwmRange(servoPort, range);
    }

    public void start() {
        in.start();
        out.start();

    }

    double pwm = 0.0;
    public void loop() {

        telemetry.addData("Intake state:", in.currentState.toString());
        telemetry.addData("Level:", out.placingLevel);
        telemetry.addData("LevelLift:", out.levelLiftPosition());
        telemetry.addData("Lift encoder:", out.getLiftPosition());
        telemetry.addData("Outtake state:", out.currentState.toString());
        myDrivetrain.updatePowers();
        telemetry.update();


        if (gamepad2.right_trigger > 0.7) {
            hook1.setPosition(0.5);
            hook2.setPosition(0.5);

        }
        else if (out.placingLevel <= 3) {
            hook1.setPosition(0.45);
            hook2.setPosition(0.45);
        }
        else {
            hook1.setPosition(0.25);
            hook2.setPosition(0.25);
        }

        if (gamepad2.left_stick_button) {
            capstone.setPosition(0);
        }

        if (gamepad2.right_stick_button) {
            capstone.setPosition(0.25);
        }



    }


}
