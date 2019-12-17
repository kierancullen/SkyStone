package org.firstinspires.ftc.teamcode;

import com.evolutionftc.autopilot.AutopilotHost;
import com.evolutionftc.autopilot.AutopilotSystem;
import com.evolutionftc.autopilot.AutopilotTracker;
import com.evolutionftc.autopilot.AutopilotTrackerDualOdo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.GlobalMovement.*;

public class AutoTests extends LinearOpMode {

    Drivetrain drivetrain;

    AutopilotHost autopilot;
    AutopilotTracker tracker;

    BNO055IMU imu;

    public static double[] ROBOT_INIT_POSITION = new double[]{0, 0, 0};
    public static double[] ROBOT_INIT_ATTITUDE = new double[]{0, 0, 0};

    public static double DUALODO_X_RADIUS;
    public static double DUALODO_Y_RADIUS;
    public static double[] DUALODO_INTERSECT_POS = new double[3];
    public static int DUALODO_TICKS_PER_UNIT; // use inches as unit


    public static int AP_COUNTS_TO_STABLE = 3;
    public static double AP_NAV_UNITS_TO_STABLE = 0.7; // inch +/-
    public static double AP_ORIENT_UNITS_TO_STABLE = 0.05; // rad +/-

    
    public void runOpMode() {
        drivetrain = new Drivetrain(
                hardwareMap.dcMotor.get("tl"),
                hardwareMap.dcMotor.get("tr"),
                hardwareMap.dcMotor.get("bl"),
                hardwareMap.dcMotor.get("br")
        );

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu.initialize(parameters);

        autopilot = new AutopilotHost(telemetry);
        tracker = new AutopilotTrackerDualOdo(
                drivetrain.getXOdometer(),
                drivetrain.getYOdometerRight(),
                DUALODO_X_RADIUS, DUALODO_Y_RADIUS, DUALODO_INTERSECT_POS,
                DUALODO_TICKS_PER_UNIT, imu
        );

        ((AutopilotTrackerDualOdo)tracker).setInverts(false, false);
        autopilot.setCountsToStable(AP_COUNTS_TO_STABLE);
        autopilot.setNavigationUnitsToStable(AP_NAV_UNITS_TO_STABLE);
        autopilot.setOrientationUnitsToStable(AP_ORIENT_UNITS_TO_STABLE);

        waitForStart();

        // record any drift while waiting, and zero it out
        autopilot.communicate(tracker);
        tracker.setRobotAttitude(ROBOT_INIT_ATTITUDE);
        tracker.setRobotPosition(ROBOT_INIT_POSITION);

        while (opModeIsActive()) {
            autopilot.communicate(tracker);
            AutopilotSystem.visualizerBroadcastRoutine(autopilot);
        }

    }

}
