package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="UserControl")
public class UserControl extends RobotOpMode2 {

    @Override
    public void init() {
        super.init();

    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();

        double horiz = (gamepad1.right_trigger * 0.5) - (gamepad1.left_trigger * 0.5);
        GlobalMovement.movement_x = horiz;
        GlobalMovement.movement_y = (-gamepad1.left_stick_y * 0.5 + -gamepad1.right_stick_y * 0.5) / 2.0;
        GlobalMovement.movement_turn = (-gamepad1.right_stick_y * 0.5) - (-gamepad1.left_stick_y * 0.5);

        in.tick(false);
        out.tick(gamepad2.right_bumper, gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.left_bumper);
        if (in.readyForGrab) { in.readyForGrab = false; } // reset; triggering done
    }
}
