package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="UserControl")
public class UserControl extends RobotOpModeSwing {

    static double LP_HORIZ_M = .25;
    static double LP_DIFF_M = .35;
    static double HP_HORIZ_M = 1.0; //0.75
    static double HP_DIFF_M = .75;

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

        double horiz;
        double l;
        double r;
        if (gamepad1.left_bumper || gamepad1.right_bumper) {
            horiz = (gamepad1.right_trigger * LP_HORIZ_M) - (gamepad1.left_trigger * LP_HORIZ_M);
            l = -gamepad1.left_stick_y * LP_DIFF_M;
            r = -gamepad1.right_stick_y * LP_DIFF_M;
        }
        else {
            horiz = (gamepad1.right_trigger * HP_HORIZ_M) - (gamepad1.left_trigger * HP_HORIZ_M);
            l = -gamepad1.left_stick_y * HP_DIFF_M;
            r = -gamepad1.right_stick_y * HP_DIFF_M;
        }

        GlobalMovement.movement_x = horiz;
        GlobalMovement.movement_y = (l + r) / 2.0;
        GlobalMovement.movement_turn = r - l;

        in.tick(false, gamepad2.back, gamepad2.right_trigger);
        out.tick(gamepad2.right_bumper, gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.left_bumper);
        if (in.readyForGrab) { in.readyForGrab = false; } // reset; triggering done
    }
}
