package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//Opmode to facilitate full-power strafing and speed measurement of the drivetrain for experimental purposes
@TeleOp(name = "SpeedTests")
public class SpeedTests extends RobotOpModeSwing {
    public void init() {
        super.init();
    }

    public void loop() {
        if (gamepad2.dpad_up) {
            myDrivetrain.updatePowersRaw(1.0, 1.0, -1.0, -1.0);
        }
        else if (gamepad2.dpad_down) {
            myDrivetrain.updatePowersRaw(-1.0, -1.0, 1.0, 1.0);
        }
        else if (gamepad2.dpad_left) {
            myDrivetrain.updatePowersRaw(-1.0, 1.0, 1.0, -1.0);
        }
        else if (gamepad2.dpad_right) {
            myDrivetrain.updatePowersRaw(1.0, -1.0, -1.0, 1.0);
        }
        else {
            myDrivetrain.updatePowersRaw(0, 0, 0, 0);
        }
    }
}
