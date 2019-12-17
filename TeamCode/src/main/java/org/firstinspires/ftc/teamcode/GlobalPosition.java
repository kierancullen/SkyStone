package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class GlobalPosition {

    public static double worldPosition_x = 0;
    public static double worldPosition_y = 0;
    public static double worldAngle_rad = 0;

    public static double lastX;
    public static double lastY;


    public static DcMotor OFS_x;
    public static DcMotor OFS_y;

    public static BNO055IMU imu;

    public static final double TICKS_PER_REVOLUTION = 1440;
    public static final double WHEEL_DIA = 4.8*Math.PI;
    public static long lastDebugTime;

    public static void updatePosition() {

        double ticksPerCM = TICKS_PER_REVOLUTION / WHEEL_DIA;

        double currentX = fetchX();
        double currentY = fetchY();
        double currentHeading = fetchHeading();

        double dX = (double)(currentX - lastX) / ticksPerCM;
        double dY = (double)(currentY - lastY) / ticksPerCM;

        double distanceTraveled = Math.hypot(dX, dY);
        double travelHeading = (((currentHeading - worldAngle_rad) % (2*Math.PI)) / 2);

        lastX = currentX;
        lastY = currentY;
        worldAngle_rad = currentHeading;











    }

    public static double fetchHeading() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        if (angles.firstAngle < 0) {
            return angles.firstAngle + (2*Math.PI);
        }

        else {
            return angles.firstAngle;
        }
    }

    public static double fetchX() {

        return OFS_x.getCurrentPosition();
    }

    public static double fetchY() {

        return OFS_y.getCurrentPosition();
    }

    public static void sendDebugPosition() {
        long currentTime = System.currentTimeMillis();
        if (currentTime-lastDebugTime > 50) {
            Log.v("AutopilotVisBcast", "stopped,"+worldPosition_x+","+worldPosition_y+","+(worldAngle_rad-(Math.PI/2)));
            lastDebugTime = currentTime;
        }

    }

}
