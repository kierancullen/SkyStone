package org.firstinspires.ftc.teamcode;

import com.evolutionftc.autopilot.AutopilotHost;
import com.evolutionftc.autopilot.AutopilotSegment;
import com.evolutionftc.autopilot.AutopilotSystem;
import com.evolutionftc.autopilot.AutopilotTracker;
import com.evolutionftc.autopilot.AutopilotTrackerTripleOdo;
import com.evolutionftc.autopilot.DiscreteIntegralAdjuster;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import static org.firstinspires.ftc.teamcode.GlobalMovement.movement_turn;
import static org.firstinspires.ftc.teamcode.GlobalMovement.movement_x;
import static org.firstinspires.ftc.teamcode.GlobalMovement.movement_y;

public class AutoCommonSideGrip extends LinearOpMode {
    public boolean intakeGo = false;
    public boolean triggerGrab = false;
    public boolean controlUp = false;
    public boolean controlDown = false;
    public boolean triggerRelease = false;
    public boolean armUp = false;
    public boolean armDown = false;
    public boolean autoPlace = false;

    PixelPopTests popper;

    Drivetrain myDrivetrain;

    DcMotor winchLeft;
    DcMotor winchRight;

    DcMotor intakeLeft;
    DcMotor intakeRight;

    Servo swingLeft;
    Servo swingRight;
    Servo arm1;
    Servo arm2;
    Servo grip;

    Servo grab1;
    Servo grab2;

    AutoIntakeController in;
    OuttakeController2 out;
    AnalogInput scotty;

    AutopilotHost autopilot;
    AutopilotTracker tracker;

    public static double[] ROBOT_INIT_POSITION = new double[]{36, 9.2, 0};
    public static double[] ROBOT_INIT_ATTITUDE = new double[]{0, 0, 0};
    public static double[] INVERT_ROBOT_INIT_POSITION = new double[]{-36, 9.2, 0};
    public static double[] INVERT_ROBOT_INIT_ATTITUDE = new double[]{0, 0, 0};

    public static double DUALODO_X_RADIUS = 3.25614173 - 2.3134252;
    public static double DUALODO_Y_RADIUS = -0.25051181102 + 7.184370097;
    public static double DUALODO_TICKS_PER_UNIT = 1440 /  (1.88976 * Math.PI);


    public static int AP_COUNTS_TO_STABLE = 10;
    public static double AP_NAV_UNITS_TO_STABLE = 1; // inch +/- //0.5
    public static double AP_ORIENT_UNITS_TO_STABLE = 0.05; // rad +/-


    public void setServoExtendedRange(Servo servo, int min, int max) {
        ServoControllerEx controller = (ServoControllerEx) servo.getController();
        int servoPort = servo.getPortNumber();
        PwmControl.PwmRange range = new PwmControl.PwmRange(min, max);
        controller.setServoPwmRange(servoPort, range);
    }

    public void apGoTo(double[] pos, double hdg, boolean useOrientation, boolean useTranslation, boolean fullStop) {
        AutopilotSegment seg = new AutopilotSegment();
        seg.navigationTarget = pos;
        seg.orientationTarget = hdg;
        seg.navigationGain = 0.015;
        seg.orientationGain = 1.25;
        seg.navigationMax = 1.0;
        seg.navigationMin = 0.25;
        seg.orientationMax = 0.5;
        seg.useOrientation = useOrientation;
        seg.useTranslation = useTranslation;
        seg.fullStop = fullStop;

        autopilot.setNavigationTarget(seg);
        autopilot.setNavigationStatus(AutopilotHost.NavigationStatus.RUNNING);

        double [] yxh = null;
        long lastTime = System.currentTimeMillis();

        while (autopilot.getNavigationStatus() == AutopilotHost.NavigationStatus.RUNNING && opModeIsActive()) {

            idleStateMachines();

            if (yxh != null) {
                /*GlobalMovement.*/movement_y = yxh[0];
                /*GlobalMovement.*/movement_x = yxh[1];
                /*GlobalMovement.*/movement_turn = yxh[2];
                myDrivetrain.updatePowers();
            }
            autopilot.communicate(tracker);

            long timeNow = System.currentTimeMillis();
            telemetry.addData("FPS", 1000.0 / (timeNow - lastTime));
            lastTime = timeNow;

            //AutopilotSystem.visualizerBroadcastRoutine(autopilot);
            autopilot.telemetryUpdate();
            telemetry.update();

            yxh = autopilot.navigationTick(tracker.getDeltaPos());
        }

    }

    public void apGoTo(double[] pos, double hdg, boolean useOrientation, boolean useTranslation, boolean fullStop, double navigationMax, double navigationMin, double navigationGain) {
        AutopilotSegment seg = new AutopilotSegment();
        seg.navigationTarget = pos;
        seg.orientationTarget = hdg;
        seg.navigationGain = navigationGain;
        seg.orientationGain = 1.25;
        seg.navigationMax = navigationMax;
        seg.navigationMin = navigationMin;
        seg.orientationMax = 0.9; //0.5
        seg.useOrientation = useOrientation;
        seg.useTranslation = useTranslation;
        seg.fullStop = fullStop;

        autopilot.setNavigationTarget(seg);
        autopilot.setNavigationStatus(AutopilotHost.NavigationStatus.RUNNING);

        double [] yxh = null;
        long lastTime = System.currentTimeMillis();

        while (autopilot.getNavigationStatus() == AutopilotHost.NavigationStatus.RUNNING && opModeIsActive()) {

            idleStateMachines();

            if (yxh != null) {
                /*GlobalMovement.*/movement_y = yxh[0];
                /*GlobalMovement.*/movement_x = yxh[1];
                /*GlobalMovement.*/movement_turn = yxh[2];
                myDrivetrain.updatePowers();
            }
            autopilot.communicate(tracker);

            long timeNow = System.currentTimeMillis();
            //telemetry.addData("FPS", 1000.0 / (timeNow - lastTime));
            lastTime = timeNow;

            //AutopilotSystem.visualizerBroadcastRoutine(autopilot);
            autopilot.telemetryUpdate();
            telemetry.update();

            yxh = autopilot.navigationTick(tracker.getDeltaPos());
        }

    }

    public void apGoToNoStrafe(double[] pos, double hdg, boolean useOrientation, boolean useTranslation, boolean fullStop, double navigationMax, double navigationMin, double navigationGain) {
        AutopilotSegment seg = new AutopilotSegment();
        seg.navigationTarget = pos;
        seg.orientationTarget = hdg;
        seg.navigationGain = navigationGain;
        seg.orientationGain = 1.25;
        seg.navigationMax = navigationMax;
        seg.navigationMin = navigationMin;
        seg.orientationMax = 0.9; //0.5
        seg.useOrientation = useOrientation;
        seg.useTranslation = useTranslation;
        seg.fullStop = fullStop;

        autopilot.setNavigationTarget(seg);
        autopilot.setNavigationStatus(AutopilotHost.NavigationStatus.RUNNING);

        double [] yxh = null;
        long lastTime = System.currentTimeMillis();

        while (autopilot.getNavigationStatus() == AutopilotHost.NavigationStatus.RUNNING && opModeIsActive()) {

            idleStateMachines();

            if (yxh != null) {
                /*GlobalMovement.*/movement_y = yxh[0];
                /*GlobalMovement.*/movement_x = yxh[1];
                if (yxh[0] < 0) {
                    /*GlobalMovement.*/movement_turn = -yxh[1];
                }
                else {
                    /*GlobalMovement.*/movement_turn = -yxh[1];
                }
                myDrivetrain.updatePowers();
            }
            autopilot.communicate(tracker);

            long timeNow = System.currentTimeMillis();
            //telemetry.addData("FPS", 1000.0 / (timeNow - lastTime));
            lastTime = timeNow;

            //AutopilotSystem.visualizerBroadcastRoutine(autopilot);
            //autopilot.telemetryUpdate();
            //telemetry.update();

            yxh = autopilot.navigationTick(tracker.getDeltaPos());
        }

    }

    public void runOpMode() {
        runOpMode(false);
    }

    public void runOpMode(boolean invert) {

        winchLeft = hardwareMap.get(DcMotor.class, "winchLeft");
        winchLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        winchRight = hardwareMap.get(DcMotor.class,"winchRight");

        winchLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winchRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotor.class, "intakeRight");
        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        swingLeft = hardwareMap.get(Servo.class, "swingLeft");
        swingRight = hardwareMap.get(Servo.class, "swingRight");
        swingRight.setDirection(Servo.Direction.REVERSE);
        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");
        arm2.setDirection(Servo.Direction.REVERSE);


        grip = hardwareMap.get(Servo.class, "grip");
        grip.setDirection(Servo.Direction.REVERSE);
        setServoExtendedRange(grip, 500, 2500);


        setServoExtendedRange(swingLeft, 500, 2500);
        setServoExtendedRange(swingRight, 500, 2500);
        setServoExtendedRange(arm1, 500, 2500);
        setServoExtendedRange(arm2, 500, 2500);

        grab1 = hardwareMap.get(Servo.class, "grab1");
        grab1.setDirection(Servo.Direction.REVERSE);
        grab2 = hardwareMap.get(Servo.class, "grab2");

        DcMotor tl = hardwareMap.get(DcMotor.class, "tl");
        DcMotor tr = hardwareMap.get(DcMotor.class, "tr");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");

        tl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        myDrivetrain = new Drivetrain (tl, tr, bl, br);

        scotty = hardwareMap.get(AnalogInput.class, "scotty");

        out = new OuttakeController2 (winchRight, winchLeft, arm1, arm2, grip);
        in = new AutoIntakeController (intakeLeft, intakeRight, swingLeft, swingRight, scotty, out);


        autopilot = new AutopilotHost(telemetry);
        tracker = new AutopilotTrackerTripleOdo(
                myDrivetrain.getXOdometer(),
                myDrivetrain.getYOdometerLeft(),
                myDrivetrain.getYOdometerRight(),
                DUALODO_X_RADIUS, DUALODO_Y_RADIUS,
                DUALODO_TICKS_PER_UNIT
        );

        ((AutopilotTrackerTripleOdo) tracker).setInverts(false, true, false);
        autopilot.setCountsToStable(AP_COUNTS_TO_STABLE);
        autopilot.setNavigationUnitsToStable(AP_NAV_UNITS_TO_STABLE);
        autopilot.setOrientationUnitsToStable(AP_ORIENT_UNITS_TO_STABLE);

        if (invert) {
            autopilot.setNavigationTargetInverts(new boolean[]{true, false, false});
            autopilot.setOrientationTargetInvert(true);
        }

        autopilot.setupVelocityPID(0,0,0,60);


        popper = new PixelPopTests();
        popper.initVuforia();
        while (!opModeIsActive()) {
            if (!invert) {
                popper.captureLocations(popper.STONE_LOCATIONS_RED);
            }
            else {
                popper.captureLocations(popper.STONE_LOCATIONS_BLUE);
            }
            if (popper.locations != null) {
                telemetry.addData("first", popper.locations[1]);
                telemetry.addData("second", popper.locations[0]);
                telemetry.update();
            }
        }

        in.start();
        out.start();

        // record any drift that happened while waiting, and zero it out
        autopilot.communicate(tracker);
        if (!invert) {
            tracker.setRobotAttitude(ROBOT_INIT_ATTITUDE);
            tracker.setRobotPosition(ROBOT_INIT_POSITION);
        }
        else {
            tracker.setRobotAttitude(INVERT_ROBOT_INIT_ATTITUDE);
            tracker.setRobotPosition(INVERT_ROBOT_INIT_POSITION);
        }

        apGoTo(new double[]{36, 81.2, 0}, 0, true, true, true, 1.0, 0.15, 0.0005);

        /*
        int location = popper.locations[1];
        if (location == 0) {
            apGoTo(new double[]{12, 36, 0}, 3*Math.PI/2, true, true, false);

        }
        if (location == 1) {
            apGoTo(new double[]{20, 36, 0}, 3*Math.PI/2, true, true, false);
        }
        if (location == 2) {
            apGoTo(new double[]{2, 36, 0}, 3*Math.PI/2, true, true, false);
        }
        if (location == 3) {
            apGoTo(new double[]{36, 36, 0}, Math.PI/6, true, true, false);
            apGoTo(new double[]{34, 40, 0}, Math.PI/6, true, true, false);
            intakeGo = true;
            apGoTo(new double[]{36, 28, 0}, Math.PI/2, true, true, false);
            triggerGrab = true;


        }
        if (location == 4) {
            apGoTo(new double[]{44, 36, 0}, Math.PI/6, true, true, false);
            apGoTo(new double[]{42, 40, 0}, Math.PI/6, true, true, false);
            intakeGo = true;
            apGoTo(new double[]{44, 28, 0}, Math.PI/2, true, true, false);
            triggerGrab = true;
        }
        if (location == 5) {
            apGoTo(new double[]{52, 36, 0}, Math.PI/6, true, true, false);
            apGoTo(new double[]{50, 40, 0}, Math.PI/6, true, true, false);
            intakeGo = true;
            apGoTo(new double[]{52, 28, 0}, Math.PI/2, true, true, false);
            triggerGrab = true;
        }

        /*
        //apGoTo(new double[]{72, 32, 0}, Math.PI/2, true, true, false, 1.0, 0.7, 0.005);
        apGoTo(new double[]{5*24 , 28, 0}, Math.PI/2, true, true, false, 0.7, 0.15, 0.03);
        autoPlace=true;
        apGoTo(new double[]{5*24 , 40, 0}, Math.PI, true, true, true, 0.5, 0.15, 0.02); //38
        grab1.setPosition(0.53);
        grab2.setPosition(0.53);
        triggerRelease=true;
        sleep(1000);
        apGoToNoStrafe(new double[]{4*24 + 1 , 14, 0}, Math.PI, false, true, false, 0.7, 0.3, 0.02);
        apGoTo(new double[]{4*24 +1 , 14, 0}, Math.PI/2, true, false, false, 0.7, 0.3, 0.02);
        apGoTo(new double[]{5*24 - 4 , 14, 0}, Math.PI/2, true, true, true, 0.7, 0.5, 0.015);
        grab1.setPosition(0.25);
        grab2.setPosition(0.25);
        sleep(1000);
        apGoTo(new double[]{3*24 , 28, 0}, Math.PI/2, true, true, true);

        /*
        autoPlace = false;
        location = 1; // popper.locations[1];
        if (location == 0) {
            apGoTo(new double[]{20 , 28, 0}, Math.PI/2, true, true, true, 0.7, 0.15, 0.015);
            apGoTo(new double[]{20, 36, 0}, Math.PI/6, true, true, false);
            apGoTo(new double[]{16, 44, 0}, Math.PI/6, true, true, false);
            intakeGo = true;
            apGoTo(new double[]{20, 28, 0}, Math.PI/2, true, true, false);
            triggerGrab = true;
        }
        if (location == 1) {
            apGoTo(new double[]{20 , 28, 0}, Math.PI/2, true, true, true, 0.7, 0.15, 0.015);
            apGoTo(new double[]{20, 36, 0}, Math.PI/6, true, true, false);
            apGoTo(new double[]{16, 44, 0}, Math.PI/6, true, true, false);
            intakeGo = true;
            apGoTo(new double[]{20, 28, 0}, Math.PI/2, true, true, false);
            triggerGrab = true;
        }
        if (location == 2) {
            apGoTo(new double[]{20, 32, 0}, 0, true, true, false);
            apGoTo(new double[]{20, 37, 0}, 0, true, true, false);
            intakeGo = true;
            apGoTo(new double[]{20, 32, 0}, 0, true, true, false);
        }
        if (location == 3) {
            apGoTo(new double[]{36, 36, 0}, Math.PI/6, true, true, false);
            apGoTo(new double[]{32, 40, 0}, Math.PI/6, true, true, false);
            intakeGo = true;
            apGoTo(new double[]{36, 28, 0}, Math.PI/2, true, true, false);
            triggerGrab = true;


        }
        if (location == 4) {
            apGoTo(new double[]{36, 32, 0}, 0, true, true, false);
            apGoTo(new double[]{36, 37, 0}, 0, true, true, false);
            intakeGo = true;
            apGoTo(new double[]{36, 32, 0}, 0, true, true, false);
        }
        if (location == 5) {
            apGoTo(new double[]{44, 32, 0}, 0, true, true, false);
            apGoTo(new double[]{44, 37, 0}, 0, true, true, false);
            intakeGo = true;
            apGoTo(new double[]{44, 32, 0}, 0, true, true, false);
        }

        apGoTo(new double[]{5*24 - 12 , 28, 0}, Math.PI/2, true, true, true, 0.7, 0.15, 0.03);
        autoPlace=true;
        sleep(1000);
        triggerRelease=true;
        apGoTo(new double[]{3*24 , 28, 0}, Math.PI/2, true, true, true, 0.7, 0.15, 0.03);



        /*
        apGoTo(new double[]{5*24 - 6, 24, 0}, 3*Math.PI/4, true, true, false, 1.0, 0.5, 0.05);
        apGoTo(new double[]{5*24 - 6 , 24, 0}, Math.PI/2, true, false, false, 1.0, 0.5, 0.05);
        //apGoTo(new double[]{5*24 -3, 24, 0}, Math.PI/2, true, true, false, 1.0, 0.5, 0.05);

        apGoTo(new double[]{2*24, 28, 0}, Math.PI/2, true, true, true, 0.7, 0.15, 0.03);

         */






        /*

        while (out.currentState != OuttakeController2.OuttakeState.HUMAN) {
            idleStateMachines();
        }
        triggerRelease = true;
        apGoTo(new double[]{5*24, 32, 0}, Math.PI, true, true, true);
        apGoTo(new double[]{72, 32, 0}, Math.PI/2, true, true, true);
        */




        while (opModeIsActive()) {
            autopilot.communicate(tracker);
            autopilot.telemetryUpdate();
            telemetry.update();
            idleStateMachines();
        }
    }

    public void idleStateMachines() {
        in.tick(intakeGo, false);
        if (intakeGo) { intakeGo = false; }
        out.tick(triggerGrab, controlUp, controlDown, triggerRelease, armUp, armDown, autoPlace);
        if (triggerGrab) { triggerGrab = false; }
        if (triggerRelease) {triggerRelease = false; }
    }
}
