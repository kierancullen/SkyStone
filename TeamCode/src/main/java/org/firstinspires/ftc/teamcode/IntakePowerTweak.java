package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="IntakePowerTweak")
public class IntakePowerTweak extends RobotOpMode {

    double currentPower = 0;
    public void init() {
        super.init();
    }

    public void loop() {

        if (gamepad1.y) currentPower += 0.05;
        if (gamepad1.a) currentPower -= 0.05;

        if (currentPower > 1) currentPower = 1;

        telemetry.addData("Intake Power:", currentPower);
        GlobalMovement.intakePower = currentPower;

        super.loop();
    }
}
