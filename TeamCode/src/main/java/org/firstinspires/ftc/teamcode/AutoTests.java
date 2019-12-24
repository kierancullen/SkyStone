package org.firstinspires.ftc.teamcode;

import com.evolutionftc.autopilot.AutopilotHost;
import com.evolutionftc.autopilot.AutopilotSegment;
import com.evolutionftc.autopilot.AutopilotSystem;
import com.evolutionftc.autopilot.AutopilotTracker;
import com.evolutionftc.autopilot.AutopilotTrackerDualOdo;
import com.evolutionftc.autopilot.AutopilotTrackerTripleOdo;
import com.evolutionftc.autopilot.DiscreteIntegralAdjuster;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import static org.firstinspires.ftc.teamcode.GlobalMovement.*;

@Autonomous(name="AutoTests")
public class AutoTests extends LinearOpMode {

    Drivetrain drivetrain;

    AutopilotHost autopilot;
    AutopilotTracker tracker;

    BNO055IMU imu;

    public static double[] ROBOT_INIT_POSITION = new double[]{0, 0, 0};
    public static double[] ROBOT_INIT_ATTITUDE = new double[]{0, 0, 0};

    public static double DUALODO_X_RADIUS = 3.25614173 - 2.3134252;
    public static double DUALODO_Y_RADIUS = -0.25051181102 + 7.184370097;
    public static double DUALODO_TICKS_PER_UNIT = 1440 /  (1.88976 * Math.PI);


    public static int AP_COUNTS_TO_STABLE = 10;
    public static double AP_NAV_UNITS_TO_STABLE = 0.5; // inch +/-
    public static double AP_ORIENT_UNITS_TO_STABLE = 0.05; // rad +/-

    public static double XY_KI;
    public static double H_KI;
    public static double MAX_RATE_X;
    public static double MAX_RATE_Y;
    public static double MAX_RATE_H;


    public void apGoTo(double[] pos, double hdg, boolean useOrientation, boolean useTranslation, boolean fullStop) {
        AutopilotSegment seg = new AutopilotSegment();
        seg.navigationTarget = pos;
        seg.orientationTarget = hdg;
        seg.navigationGain = 0.015;
        seg.orientationGain = 1.25;
        seg.navigationMax = 0.50;
        seg.navigationMin = 0.15;
        seg.orientationMax = 0.25;
        seg.useOrientation = useOrientation;
        seg.useTranslation = useTranslation;
        seg.fullStop = fullStop;

        autopilot.setNavigationTarget(seg);
        autopilot.setNavigationStatus(AutopilotHost.NavigationStatus.RUNNING);

        double [] yxh = null;
        long lastTime = System.currentTimeMillis();

        DiscreteIntegralAdjuster adjY = new DiscreteIntegralAdjuster(XY_KI);
        DiscreteIntegralAdjuster adjX = new DiscreteIntegralAdjuster(XY_KI);
        DiscreteIntegralAdjuster adjH = new DiscreteIntegralAdjuster(H_KI);

        while (autopilot.getNavigationStatus() == AutopilotHost.NavigationStatus.RUNNING && opModeIsActive()) {

            if (yxh != null) {
                /*GlobalMovement.*/movement_y = adjY.adjust(yxh[0], tracker.getRateY() / MAX_RATE_X);
                /*GlobalMovement.*/movement_x = adjX.adjust(yxh[1], tracker.getRateX() / MAX_RATE_Y);
                /*GlobalMovement.*/movement_turn = adjH.adjust(yxh[2], tracker.getRateH() / MAX_RATE_H);
                drivetrain.updatePowers();
            }
            autopilot.communicate(tracker);

            long timeNow = System.currentTimeMillis();
            telemetry.addData("FPS", 1000.0 / (timeNow - lastTime));
            lastTime = timeNow;

            AutopilotSystem.visualizerBroadcastRoutine(autopilot);
            autopilot.telemetryUpdate();
            telemetry.update();

            yxh = autopilot.navigationTick();
        }
    }


    public void runOpMode() {
        drivetrain = new Drivetrain(
                hardwareMap.dcMotor.get("tl"),
                hardwareMap.dcMotor.get("tr"),
                hardwareMap.dcMotor.get("bl"),
                hardwareMap.dcMotor.get("br")
        );
        /*drivetrain.topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drivetrain.topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drivetrain.bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drivetrain.bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu.initialize(parameters);

        autopilot = new AutopilotHost(telemetry);
        tracker = new AutopilotTrackerTripleOdo(
                drivetrain.getXOdometer(),
                drivetrain.getYOdometerLeft(),
                drivetrain.getYOdometerRight(),
                DUALODO_X_RADIUS, DUALODO_Y_RADIUS,
                DUALODO_TICKS_PER_UNIT
        );

        ((AutopilotTrackerTripleOdo)tracker).setInverts(false, true, false);
        autopilot.setCountsToStable(AP_COUNTS_TO_STABLE);
        autopilot.setNavigationUnitsToStable(AP_NAV_UNITS_TO_STABLE);
        autopilot.setOrientationUnitsToStable(AP_ORIENT_UNITS_TO_STABLE);

        waitForStart();

        // record any drift that happened while waiting, and zero it out
        autopilot.communicate(tracker);
        tracker.setRobotAttitude(ROBOT_INIT_ATTITUDE);
        tracker.setRobotPosition(ROBOT_INIT_POSITION);


        apGoTo(new double[]{0, 0, 0}, Math.PI, true, false, true);

        movement_x=0;
        movement_y=0;
        movement_turn=0;
        drivetrain.updatePowers();

        while (opModeIsActive()) {
            autopilot.communicate(tracker);
            autopilot.telemetryUpdate();
            telemetry.update();
            AutopilotSystem.visualizerBroadcastRoutine(autopilot);
        }

    }

}
