package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.GlobalMovement.movement_turn;
import static org.firstinspires.ftc.teamcode.GlobalMovement.movement_x;
import static org.firstinspires.ftc.teamcode.GlobalMovement.movement_y;

public class Drivetrain {

    public DcMotor topLeft, topRight, bottomLeft, bottomRight;

    public Drivetrain(DcMotor tl, DcMotor tr, DcMotor bl, DcMotor br) {
        topLeft = tl;
        topRight = tr;
        bottomLeft = bl;
        bottomRight = br;

        topLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void updatePowers() {

        double tl_power_raw = movement_y-movement_turn+movement_x*1.5;
        double bl_power_raw = movement_y-movement_turn- movement_x*1.5;
        double br_power_raw = -(movement_y+movement_turn+movement_x*1.5);
        double tr_power_raw = -(movement_y+movement_turn-movement_x*1.5);


        double maxRawPower = Math.abs(tl_power_raw);
        if(Math.abs(bl_power_raw) > maxRawPower){ maxRawPower = Math.abs(bl_power_raw);}
        if(Math.abs(br_power_raw) > maxRawPower){ maxRawPower = Math.abs(br_power_raw);}
        if(Math.abs(tr_power_raw) > maxRawPower){ maxRawPower = Math.abs(tr_power_raw);}

        //if the maximum is greater than 1, scale all the powers down to preserve the shape
        double scaleDownAmount = 1.0;
        if(maxRawPower > 1.0){
            //when max power is multiplied by this ratio, it will be 1.0, and others less
            scaleDownAmount = 1.0/maxRawPower;
        }
        tl_power_raw *= scaleDownAmount;
        bl_power_raw *= scaleDownAmount;
        br_power_raw *= scaleDownAmount;
        tr_power_raw *= scaleDownAmount;

        topLeft.setPower(tl_power_raw);
        bottomLeft.setPower(bl_power_raw);
        bottomRight.setPower(br_power_raw);
        topRight.setPower(tr_power_raw);
    }

    public DcMotor getXOdometer() {
        return topLeft;
    }

    public DcMotor getYOdometerRight() {
        return bottomRight;
    }

    public DcMotor getYOdometerLeft() {
        return bottomLeft;
    }

}
