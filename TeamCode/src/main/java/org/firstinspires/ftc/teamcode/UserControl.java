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
        in.tick(gamepad2.right_bumper);
        out.tick(in.readyForGrab, gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.left_bumper);
        if (in.readyForGrab) { in.readyForGrab = false; } // reset; triggering done
    }
}
