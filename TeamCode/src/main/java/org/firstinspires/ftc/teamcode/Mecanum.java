package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.GlobalPosition.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.GlobalPosition.worldPosition_x;
import static org.firstinspires.ftc.teamcode.GlobalPosition.worldPosition_y;
import static org.firstinspires.ftc.teamcode.RobotMovement.goToPosition;

@TeleOp(name="Mecanum", group="Mecanum")
public class Mecanum extends RobotOpMode {

    DcMotor winchLeft;
    DcMotor winchRight;
    DcMotor intakeLeft;
    DcMotor intakeRight;
    Servo swing;
    Servo tilt;
    Servo grab;
    GrabLiftPlaceController g;

    @Override
    public void init() {

        super.init();
         winchLeft = hardwareMap.get(DcMotor.class, "winchLeft");
         winchRight = hardwareMap.get(DcMotor.class,"winchRight");
        intakeLeft = hardwareMap.get(DcMotor.class, "intakeL");
        intakeRight = hardwareMap.get(DcMotor.class, "intakeR");

         winchLeft.setDirection(DcMotorSimple.Direction.REVERSE);
         winchLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         winchRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

       intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);

       intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
       intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);



         swing = hardwareMap.get(Servo.class, "swing");
         tilt = hardwareMap.get(Servo.class, "tilt");
         grab = hardwareMap.get(Servo.class, "grab");

        g = new GrabLiftPlaceController(winchLeft, winchRight, swing, tilt, grab);

    }

    public void start() {
        g.start();
    }

    int servoIndex = 0;
    int motorIndex = 0;

    double pwm = 0;
    double motorPower = 0.2;
    int targetPos = 0;


    double currentPower = 0;

    public void loop() {
        GlobalMovement.updateFromGamepad(gamepad1);
        g.tick(gamepad2.right_bumper, gamepad2.a, gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.right_bumper, gamepad2.a, gamepad2.back);


        if (gamepad2.left_bumper) {
            intakeLeft.setPower(-0.45);
            intakeRight.setPower(-0.45);
        }
        else if (g.liftIsInDownPosition()) {

            if (gamepad1.y) currentPower += 0.05;
            if (gamepad1.a) currentPower -= 0.05;

            if (currentPower > 1) currentPower = 1;

            telemetry.addData("Intake Power:", currentPower);

            intakeLeft.setPower(currentPower);
            intakeRight.setPower(currentPower);
        }

        super.loop();

    }
}
