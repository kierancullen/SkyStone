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

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.teamcode.GlobalMovement.movement_turn;
import static org.firstinspires.ftc.teamcode.GlobalMovement.movement_x;
import static org.firstinspires.ftc.teamcode.GlobalMovement.movement_y;

public class AutoCommonZooming extends LinearOpMode {
    public boolean intakeGo;
    public boolean triggerGrab;
    public boolean controlUp;
    public boolean controlDown;
    public boolean triggerRelease;
    public boolean armUp;
    public boolean armDown;
    public boolean autoPlace = false;

    PixelPopNoLens popper;

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
    Servo capstone;

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
    public static double DUALODO_Y_RADIUS = 7.3204370097; //7.304370097; //-0.25051181102 + 7.184370097; //7.28370097;//
    public static double DUALODO_X_OFFSET = 7.687480315;
    public static double DUALODO_Y_OFFSET = 2.21;

    public static double DUALODO_TICKS_PER_UNIT = 1440 /  (2.5 * Math.PI); //1440 /  (1.88976 * Math.PI);


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

            yxh = autopilot.navigationTick();
        }

    }

    public void apGoTo(double[] pos, double hdg, boolean useOrientation, boolean useTranslation, boolean fullStop, double navigationMax, double navigationMin, double navigationGain) {
        apGoTo(pos, hdg, useOrientation, useTranslation, fullStop, navigationMax, navigationMin, navigationGain, 1.25, AP_NAV_UNITS_TO_STABLE, false);
    }

    public void apGoTo(double[] pos, double hdg, boolean useOrientation, boolean useTranslation, boolean fullStop, double navigationMax, double navigationMin, double navigationGain, double orientationGain, double navigationUnitsToStable, boolean diffMode) {
        AutopilotSegment seg = new AutopilotSegment();
        seg.navigationTarget = pos;
        seg.orientationTarget = hdg;
        seg.navigationGain = navigationGain;
        seg.orientationGain = orientationGain;
        seg.navigationMax = navigationMax;
        seg.navigationMin = navigationMin;
        seg.orientationMax = 0.9; //0.5
        seg.useOrientation = useOrientation;
        seg.useTranslation = useTranslation;
        seg.fullStop = fullStop;
        seg.diffMode = diffMode;

        autopilot.setNavigationUnitsToStable(navigationUnitsToStable);

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
            //autopilot.telemetryUpdate();
            //telemetry.update();

            yxh = autopilot.navigationTick();
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

            yxh = autopilot.navigationTick();
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

        //Arm
        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");
        arm2.setDirection(Servo.Direction.REVERSE);
        setServoExtendedRange(arm1, 500, 2500);
        setServoExtendedRange(arm2, 500, 2500);


        //Grip
        grip = hardwareMap.get(Servo.class, "grip");
        grip.setDirection(Servo.Direction.REVERSE);
        setServoExtendedRange(grip, 500, 2500);


        setServoExtendedRange(swingLeft, 500, 2500);
        setServoExtendedRange(swingRight, 500, 2500);

        grab1 = hardwareMap.get(Servo.class, "hook1");
        grab1.setDirection(Servo.Direction.REVERSE);
        grab2 = hardwareMap.get(Servo.class, "hook2");

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

        capstone = hardwareMap.get(Servo.class, "capstone");
        capstone.setPosition(0);


        autopilot = new AutopilotHost(telemetry);
        tracker = new AutopilotTrackerTripleOdo(
                myDrivetrain.getXOdometer(),
                myDrivetrain.getYOdometerLeft(),
                myDrivetrain.getYOdometerRight(),
                DUALODO_X_RADIUS, DUALODO_Y_RADIUS,
                DUALODO_X_OFFSET, DUALODO_Y_OFFSET,
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

        // record any drift that happened while waiting, and zero it out
        //autopilot.communicate(tracker);
        if (!invert) {
            tracker.setRobotAttitude(ROBOT_INIT_ATTITUDE);
            tracker.setRobotPosition(ROBOT_INIT_POSITION);
        }
        else {
            tracker.setRobotAttitude(INVERT_ROBOT_INIT_ATTITUDE);
            tracker.setRobotPosition(INVERT_ROBOT_INIT_POSITION);
        }
        //0autopilot.communicate(tracker);

        popper = new PixelPopNoLens();
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

        int location = popper.locations[0];
        tl.setZeroPowerBehavior(FLOAT);
        tr.setZeroPowerBehavior(FLOAT);
        br.setZeroPowerBehavior(FLOAT);
        bl.setZeroPowerBehavior(FLOAT);

        tracker.setRobotAttitude(new double[]{0,0,0});

        while (opModeIsActive()) {
            autopilot.communicate(tracker);
            GlobalMovement.updateFromGamepad(gamepad1);
            myDrivetrain.updatePowers();
            //AutopilotSystem.visualizerBroadcastRoutine(autopilot);
            autopilot.telemetryUpdate();
            telemetry.update();
        }

        //Only going for the first skystone on this run
        if (location == 0) {


        }
        if (location == 1) {

        }
        if (location == 2) {
            apGoTo(new double[]{26,40,0}, Math.PI/6, true, true, false, 0.7, 0.2, 0.03, 1.25, 1, true);
            apGoTo(new double[]{2*24,36,0}, Math.PI/2, true, true, true, 1.0, 0.7, 0.03, 1.25, 1, true);
            apGoTo(new double[]{4*24,36,0}, Math.PI/2, true, true, false, 1.0, 1.0, 0.03, 1.25, 1, false);
            apGoTo(new double[]{5*24,40,0}, Math.PI, true, true, true, 1.0, 0.2, 0.03, 1.25, 1, false);
            //apGoTo(new double[]{3*24,36,0}, Math.PI/2, true, true, false, 0.7, 0.2, 0.03, 1, true);

            //apGoTo(new double[]{4*24 - 6,36,0}, Math.PI/2, true, true, false, 0.7, 0.2, 0.03, 1, true);


            tl.setZeroPowerBehavior(BRAKE);
            tr.setZeroPowerBehavior(BRAKE);
            br.setZeroPowerBehavior(BRAKE);
            bl.setZeroPowerBehavior(BRAKE);

            tr.setPower(0);
            tl.setPower(0);
            bl.setPower(0);
            br.setPower(0);

        }

        /*
        //Drive to foundation

        autoPlace=true;

        //Bump against foundation and grab
        apGoTo(new double[]{5*24 , 44, 0}, Math.PI, true, true, true, 0.5, 0.3, 0.02, 1); //38
        grab1.setPosition(0.53);
        grab2.setPosition(0.53);
        triggerRelease=true;
        sleep(1000);

        //Pull foundation out
        apGoToNoStrafe(new double[]{4*24 + 1 , 20, 0}, Math.PI, false, true, false, 0.7, 0.3, 0.02);
        //Correct rotation and raise hooks
        apGoTo(new double[]{4*24 +1 , 20, 0}, Math.PI/2, true, false, false, 0.7, 0.3, 0.02);
        grab1.setPosition(0.25);
        grab2.setPosition(0.25);

        //Strafe to center the robot on the foundation
        apGoTo(new double[]{4*24 - 5 , 24, 0}, Math.PI/2, true, true, false, 1.0, 1.0, 0.05, 3);
        apGoTo(new double[]{5*24 - 4 , 24, 0}, Math.PI/2, true, true, false, 0.7, 0.7, 0.03, 2);
        apGoTo(new double[]{3*24 , 32, 0}, Math.PI/2, true, true, false, 0.7, 0.7, 0.03, 3);


        autoPlace = false;
        location =  popper.locations[0];
        if (location == 0) {
            apGoTo(new double[]{12 , 28, 0}, Math.PI/2, true, true, true, 0.7, 0.2, 0.015);
            apGoTo(new double[]{12, 36, 0}, Math.PI/6, true, true, false, 1.0, 0.25, 0.015, 2);
            apGoTo(new double[]{8, 54, 0}, Math.PI/6, true, true, false,1.0, 0.25, 0.015, 2);
            intakeGo = true;
            apGoTo(new double[]{12, 36, 0}, Math.PI/2, true, true, false,1.0, 0.25, 0.015, 2);
            triggerGrab = true;

        }

        if (location == 1) {
            apGoTo(new double[]{20 , 28, 0}, Math.PI/2, true, true, true, 0.7, 0.2, 0.015);
            apGoTo(new double[]{20, 36, 0}, Math.PI/6, true, true, false, 1.0, 0.25, 0.015, 2);
            apGoTo(new double[]{16, 54, 0}, Math.PI/6, true, true, false,1.0, 0.25, 0.015, 2);
            intakeGo = true;
            apGoTo(new double[]{20, 36, 0}, Math.PI/2, true, true, false,1.0, 0.25, 0.015, 2);
            triggerGrab = true;
        }
        if (location == 2) {
            apGoTo(new double[]{28 , 28, 0}, Math.PI/2, true, true, true, 0.7, 0.2, 0.015);
            apGoTo(new double[]{28, 36, 0}, Math.PI/6, true, true, false, 1.0, 0.25, 0.015, 2);
            apGoTo(new double[]{24, 54, 0}, Math.PI/6, true, true, false,1.0, 0.25, 0.015, 2);
            intakeGo = true;
            apGoTo(new double[]{28, 36, 0}, Math.PI/2, true, true, false,1.0, 0.25, 0.015, 2);
            triggerGrab = true;
        }
        if (location == 3) {
            apGoTo(new double[]{36 , 28, 0}, Math.PI/2, true, true, true, 0.7, 0.2, 0.015);
            apGoTo(new double[]{36, 36, 0}, Math.PI/6, true, true, false, 1.0, 0.25, 0.015, 2);
            apGoTo(new double[]{32, 54, 0}, Math.PI/6, true, true, false,1.0, 0.25, 0.015, 2);
            intakeGo = true;
            apGoTo(new double[]{36, 36, 0}, Math.PI/2, true, true, false,1.0, 0.25, 0.015, 2);
            triggerGrab = true;

        }
        if (location == 4) {
            apGoTo(new double[]{44 , 28, 0}, Math.PI/2, true, true, true, 0.7, 0.2, 0.015);
            apGoTo(new double[]{44, 36, 0}, Math.PI/6, true, true, false, 1.0, 0.25, 0.015, 2);
            apGoTo(new double[]{40, 54, 0}, Math.PI/6, true, true, false,1.0, 0.25, 0.015, 2);
            intakeGo = true;
            apGoTo(new double[]{44, 36, 0}, Math.PI/2, true, true, false,1.0, 0.25, 0.015, 2);
            triggerGrab = true;
        }
        if (location == 5) {
            apGoTo(new double[]{52 , 28, 0}, Math.PI/2, true, true, true, 0.7, 0.2, 0.015);
            apGoTo(new double[]{52, 36, 0}, Math.PI/6, true, true, false, 1.0, 0.25, 0.015, 2);
            apGoTo(new double[]{48, 54, 0}, Math.PI/6, true, true, false,1.0, 0.25, 0.015, 2);
            intakeGo = true;
            apGoTo(new double[]{52, 36, 0}, Math.PI/2, true, true, false,1.0, 0.25, 0.015, 2);
            triggerGrab = true;
        }

        apGoTo(new double[]{5*24 - 18 , 36, 0}, Math.PI/2, true, true, false, 0.7, 0.5, 0.03, 5);
        autoPlace=true;
        apGoTo(new double[]{5*24 - 9 , 36, 0}, Math.PI/2, true, true, true, 0.7, 0.5, 0.03, 5);
        sleep(2000);
        triggerRelease=true;
        while(out.currentState != OuttakeController2.OuttakeState.RELEASING) idleStateMachines();
        sleep(1000);
        apGoTo(new double[]{3*24 , 36, 0}, Math.PI/2, true, true, true, 0.7, 0.5, 0.03, 3); //28
        tl.setZeroPowerBehavior(BRAKE);
        tr.setZeroPowerBehavior(BRAKE);
        br.setZeroPowerBehavior(BRAKE);
        bl.setZeroPowerBehavior(BRAKE);

        tr.setPower(0);
        tl.setPower(0);
        bl.setPower(0);
        br.setPower(0);

        /*
        apGoTo(new double[]{3*24 , 28, 0}, Math.PI/2, true, true, true, 0.7, 0.5, 0.03, 2);




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
        in.tick(intakeGo, false, false);
        if (intakeGo) { intakeGo = false; }
        out.tick(triggerGrab, controlUp, controlDown, triggerRelease, armUp, armDown, autoPlace, false, false, false);
        if (triggerGrab) { triggerGrab = false; }
        if (triggerRelease) {triggerRelease = false; }
        if (autoPlace) {autoPlace = false;}
    }
}
