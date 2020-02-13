package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    Servo grab1;
    Servo grab2;

    Servo redGripSwing;
    Servo redGrip;

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

        //swingLeft.setPosition(0.06);
        //swingRight.setPosition(0.06);

        grip = hardwareMap.get(Servo.class, "grip");

        grip.setDirection(Servo.Direction.REVERSE);

        setServoExtendedRange(grip, 500, 2500);

        grab1 = hardwareMap.get(Servo.class, "grab1");
        grab1.setDirection(Servo.Direction.REVERSE);
        grab2 = hardwareMap.get(Servo.class, "grab2");

        grab1.setPosition(0.0);
        grab2.setPosition(0.0);

        redGrip = hardwareMap.get(Servo.class, "redGrip");
        redGripSwing = hardwareMap.get(Servo.class, "redGripSwing");
        redGripSwing.setDirection(Servo.Direction.REVERSE);
        redGrip.setDirection(Servo.Direction.REVERSE);
        redGrip.setPosition(0.158);

        setServoExtendedRange(redGrip, 500, 2500);
        setServoExtendedRange(redGripSwing, 500, 2500);


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
        in = new IntakeController (intakeLeft, intakeRight, swingLeft, swingRight, scotty, out);


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

        myDrivetrain.updatePowers();
        telemetry.update();
        if (gamepad2.a) {
            grab1.setPosition(0.5);
            grab2.setPosition(0.5);
        }
        if (gamepad2.y) {
            grab1.setPosition(0.25);
            grab2.setPosition(0.25);
        }

        if (gamepad2.dpad_left) redGripSwing.setPosition(0);
        if (gamepad2.dpad_right) redGripSwing.setPosition(0.32);

        if (gamepad2.right_stick_button) redGrip.setPosition(0);
        if (gamepad2.left_stick_button) redGrip.setPosition(0.158);





        telemetry.addData("Grab:", pwm);




    }


}
