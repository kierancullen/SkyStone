package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {

    public DcMotor left;
    public DcMotor right;

    public Intake (DcMotor left, DcMotor right) {
        this.right = right;
        this.left = left;

        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void updatePowers() {

        left.setPower(GlobalMovement.intakePower);
        right.setPower(GlobalMovement.intakePower);

    }
}
