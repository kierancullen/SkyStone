package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

@TeleOp(name="IntakePowerTweak")
public class IntakePowerTweak extends RobotOpMode {

    Servo swingLeft;
    Servo swingRight;
    DcMotor intakeL;
    DcMotor intakeR;

    double currentPower = 0;
    public void init() {
        super.init();

        intakeL = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeR = hardwareMap.get(DcMotor.class, "intakeRight");
        intakeL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        swingLeft = hardwareMap.get(Servo.class, "swingLeft");
        swingLeft.setDirection(Servo.Direction.REVERSE);
        swingRight = hardwareMap.get(Servo.class, "swingRight");

        setServoExtendedRange(swingLeft, 500, 2500);
        setServoExtendedRange(swingRight, 500, 2500);
        swingLeft.setPosition(0.12);
        swingRight.setPosition(0.14);
    }

    public void setServoExtendedRange(Servo servo, int min, int max) {
        ServoControllerEx controller = (ServoControllerEx) servo.getController();
        int servoPort = servo.getPortNumber();
        PwmControl.PwmRange range = new PwmControl.PwmRange(min, max);
        controller.setServoPwmRange(servoPort, range);
    }

    public void loop() {

        if (gamepad1.y) currentPower += 0.01;
        if (gamepad1.a) currentPower -= 0.01;

        if (currentPower > 1) currentPower = 1;

        intakeL.setPower(-currentPower);
        intakeR.setPower(currentPower);

        telemetry.addData("Intake Power:", currentPower);

    }
}
