package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.Gamepad;



public class GlobalMovement {
    public static double movement_x = 0;
    public static double movement_y = 0;
    public static double movement_turn = 0;

    public static double intakePower = 0;

    public static void updateFromGamepad(Gamepad myGamepad) {
        double masterScale;
        if (myGamepad.right_bumper) {
            masterScale = 0.2;
        }
        else masterScale = 0.5;
        movement_y = -myGamepad.right_stick_y * masterScale;
        movement_x = myGamepad.right_stick_x * masterScale;
        movement_turn = -((myGamepad.left_stick_x * masterScale)); // + (myGamepad.right_stick_x * 0.5));




    }
}
