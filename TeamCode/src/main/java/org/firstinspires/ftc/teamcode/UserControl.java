package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="UserControl")
public class UserControl extends RobotOpModeSwing {

    static double LP_HORIZ_M = .5;
    static double LP_DIFF_M = .35;
    static double HP_HORIZ_M = 1.0; //0.75
    static double HP_DIFF_M = 1.0;

    final boolean CONDENSED = false;
    boolean triggerGrab = false;
    boolean stowed = false;

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
        if (!CONDENSED) {
            if (gamepad1.left_bumper || gamepad1.right_bumper) {
                horiz = 1.5 * (gamepad1.right_trigger * LP_HORIZ_M) - (gamepad1.left_trigger * LP_HORIZ_M);
                l = -gamepad1.left_stick_y * LP_DIFF_M;
                r = -gamepad1.right_stick_y * LP_DIFF_M;
            } else {
                horiz = 1.5 * (gamepad1.right_trigger * HP_HORIZ_M) - (gamepad1.left_trigger * HP_HORIZ_M);
                l = -gamepad1.left_stick_y * HP_DIFF_M;
                r = -gamepad1.right_stick_y * HP_DIFF_M;
            }
        }

        else {
            if (gamepad2.left_bumper || gamepad2.right_bumper) {
                horiz = 1.5 * (gamepad2.right_trigger * LP_HORIZ_M) - (gamepad2.left_trigger * LP_HORIZ_M);
                l = -gamepad2.left_stick_y * LP_DIFF_M;
                r = -gamepad2.right_stick_y * LP_DIFF_M;
            } else {
                horiz = 1.5 * (gamepad2.right_trigger * HP_HORIZ_M) - (gamepad2.left_trigger * HP_HORIZ_M);
                l = -gamepad2.left_stick_y * HP_DIFF_M;
                r = -gamepad2.right_stick_y * HP_DIFF_M;
            }
        }


        GlobalMovement.movement_x = horiz;
        GlobalMovement.movement_y = (l + r) / 2.0;
        GlobalMovement.movement_turn = (r - l) / 3;
        triggerGrab=false;
        if (gamepad2.right_bumper) triggerGrab = true;
        in.tick(false, gamepad2.back, stowed);
        out.tick(triggerGrab, gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.left_bumper, gamepad2.x, gamepad2.b, false, gamepad2.dpad_left, gamepad2.left_trigger > 0.1, gamepad2.right_trigger > 0.1);
        if (triggerGrab) triggerGrab = false;

    }
}
