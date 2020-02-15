package org.firstinspires.ftc.teamcode;

import com.evolutionftc.autopilot.AutopilotHost;
import com.evolutionftc.autopilot.AutopilotSegment;
import com.evolutionftc.autopilot.AutopilotTracker;
import com.evolutionftc.autopilot.AutopilotTrackerTripleOdo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import static org.firstinspires.ftc.teamcode.GlobalMovement.movement_turn;
import static org.firstinspires.ftc.teamcode.GlobalMovement.movement_x;
import static org.firstinspires.ftc.teamcode.GlobalMovement.movement_y;

public class AutoCommonSideGrip extends LinearOpMode {

    double GRABBING_Y = 39;
    double FIRST_STONE_X = 10;

    double BRIDGE_SAFE_Y = 28;
    double BRIDGE_SAFE_X = 3 * 24;

    double PLACING_Y = GRABBING_Y;
    double FIRST_PLACING_X = 5 * 24;

    int MAX_TRIPS = 6;


    public boolean intakeGo = false;
    public boolean intakeRetract = true;

    public boolean triggerGrab = false;
    public boolean controlUp = false;
    public boolean controlDown = false;
    public boolean triggerRelease = false;
    public boolean armUp = false;
    public boolean armDown = false;
    public boolean autoPlace = false;

    public boolean triggerSideGrab = false;
    public boolean triggerSideRelease = false;

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

    Servo hook1;
    Servo hook2;

    Servo redGrip;
    Servo redGripSwing;
    Servo blueGrip;
    Servo blueGripSwing;

    AutoIntakeController in;
    OuttakeController2 out;
    SideGripController side;
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
    public static double AP_NAV_UNITS_TO_STABLE = 0.5; // inch +/- //0.5
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
        seg.orientationGain = 1;
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

            if (!fullStop) {
                autopilot.setNavigationUnitsToStable(6);
                autopilot.setOrientationUnitsToStable(Math.PI/4);
            }

            else {
                autopilot.setNavigationUnitsToStable(AP_NAV_UNITS_TO_STABLE);
                autopilot.setOrientationUnitsToStable(AP_ORIENT_UNITS_TO_STABLE);
            }
        }

    }

    public void apGoToNoStrafe(double[] pos, double hdg, boolean useOrientation, boolean useTranslation, boolean fullStop, double navigationMax, double navigationMin, double navigationGain) {
        AutopilotSegment seg = new AutopilotSegment();
        seg.navigationTarget = pos;
        seg.orientationTarget = hdg;
        seg.navigationGain = navigationGain;
        seg.orientationGain = 1.0; //1.25
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

        //Winches
        winchLeft = hardwareMap.get(DcMotor.class, "winchLeft");
        winchLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        winchRight = hardwareMap.get(DcMotor.class, "winchRight");

        winchLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winchRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Intake
        intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotor.class, "intakeRight");
        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        swingLeft = hardwareMap.get(Servo.class, "swingLeft");
        swingRight = hardwareMap.get(Servo.class, "swingRight");
        swingRight.setDirection(Servo.Direction.REVERSE);
        setServoExtendedRange(swingLeft, 500, 2500);
        setServoExtendedRange(swingRight, 500, 2500);

        scotty = hardwareMap.get(AnalogInput.class, "scotty");

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

        //Foundation Hooks
        hook1 = hardwareMap.get(Servo.class, "hook1");
        hook1.setDirection(Servo.Direction.REVERSE);
        hook2 = hardwareMap.get(Servo.class, "hook2");

        //Side Grabbers
        redGrip = hardwareMap.get(Servo.class, "redGrip");
        redGripSwing = hardwareMap.get(Servo.class, "redGripSwing");
        redGripSwing.setDirection(Servo.Direction.REVERSE);
        redGrip.setDirection(Servo.Direction.REVERSE);

        setServoExtendedRange(redGrip, 500, 2500);
        setServoExtendedRange(redGripSwing, 500, 2500);

        blueGrip = hardwareMap.get(Servo.class, "blueGrip");
        blueGripSwing = hardwareMap.get(Servo.class, "blueGripSwing");


        //Drivetrain
        DcMotor tl = hardwareMap.get(DcMotor.class, "tl");
        DcMotor tr = hardwareMap.get(DcMotor.class, "tr");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");

        tl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        myDrivetrain = new Drivetrain(tl, tr, bl, br);
        movement_x = 0;
        movement_y = 0;
        movement_turn = 0;
        tl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        myDrivetrain.updatePowers();


        //State Machines
        out = new OuttakeController2(winchRight, winchLeft, arm1, arm2, grip);
        in = new AutoIntakeController(intakeLeft, intakeRight, swingLeft, swingRight, scotty, out);
        side = new SideGripController(redGrip, redGripSwing, blueGrip, blueGripSwing, invert);


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


        if (invert) {
            autopilot.setNavigationTargetInverts(new boolean[]{true, false, false});
            autopilot.setOrientationTargetInvert(true);
        }

        //autopilot.setupVelocityPID(0.1,0,0,60);


        popper = new PixelPopTests();
        popper.initVuforia();
        while (!opModeIsActive()) {
            if (!invert) {
                popper.captureLocations(popper.STONE_LOCATIONS_RED);
            } else {
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
        side.start();

        // record any drift that happened while waiting, and zero it out
        autopilot.communicate(tracker);
        if (!invert) {
            tracker.setRobotAttitude(ROBOT_INIT_ATTITUDE);
            tracker.setRobotPosition(ROBOT_INIT_POSITION);
        } else {
            tracker.setRobotAttitude(INVERT_ROBOT_INIT_ATTITUDE);
            tracker.setRobotPosition(INVERT_ROBOT_INIT_POSITION);
        }


        int[] pickingOrder = new int[6];
        pickingOrder[0] = 0; //popper.locations[0];
        pickingOrder[1] = 3; //popper.locations[1];

        //Put the rest of the stones in the array, with the shortest trips first
        for (int i = 5, j = 2; i >= 0; i--) {
            //If this is not a SkyStone, put it in the array
            if (i != popper.locations[0] && i != popper.locations[1]) {
                pickingOrder[j] = i;
                j++;
            }
        }

        for (int trip = 0; trip < MAX_TRIPS; trip++) {
            int location = pickingOrder[trip];
            //On the first trip, we need to do a special zoom-out
            if (trip == 0) {
                apGoTo(new double[]{36, GRABBING_Y - 5, 0}, 0, true, true, false, 1.0, 0.20, 0.015);
                apGoTo(new double[]{36, GRABBING_Y - 5, 0}, 3 * Math.PI / 2, true, false, false, 0.5, 0.20, 0.015);
                apGoTo(new double[]{FIRST_STONE_X + (location * 8), GRABBING_Y, 0}, 3 * Math.PI / 2, true, true, true, 1.0, 0.20, 0.011);
                side.safeToMove = false; //Not sure if this is necessary
                triggerSideGrab = true;
                while (!side.safeToMove) {
                    idleStateMachines();
                    sleep(1);
                }
            }

            //Otherwise, zoom regularly because we're coming from the foundation
            else {
                apGoTo(new double[]{FIRST_STONE_X + (location * 8), GRABBING_Y, 0}, 3 * Math.PI / 2, true, true, true, 1.0, 0.15, 0.03);
                side.safeToMove = false;
                triggerSideGrab = true;
                while (!side.safeToMove) sleep(1);
            }
            //Go to last stone location, unless we're already there
            if (location != 4 && location != 5) {
                location = 4;
                apGoTo(new double[]{FIRST_STONE_X + (location * 8), GRABBING_Y, 0}, 3 * Math.PI / 2, true, true, false, 1.0, 1.0, 0.03);


            }

            //Go to the bridge-avoidance position
            apGoTo(new double[]{BRIDGE_SAFE_X, BRIDGE_SAFE_Y, 0}, 3 * Math.PI / 2, false, true, false, 1.0, 1.0, 0.03);


            //Go to the first placing position
            apGoTo(new double[]{FIRST_PLACING_X, PLACING_Y, 0}, 3*Math.PI/2, true, true, true, 0.5, 0.2, 0.03);
            triggerSideRelease = true;
            while (!side.safeToMove) sleep(1);

            //location  = 3;
            //apGoTo(new double[]{FIRST_STONE_X + (location * 8), GRABBING_Y, 0}, 3 * Math.PI / 2, true, true, true, 0.3, 0.2, 0.03);


            }



        }
        /*

        //Drive up against foundation and latch
        apGoTo(new double[]{5*24 , 40, 0}, Math.PI, true, true, true, 0.5, 0.15, 0.02); //38
        hook1.setPosition(0.53);
        hook2.setPosition(0.53);
        sleep(1000);

        //Back up in an arc to turn the foundation
        apGoToNoStrafe(new double[]{4*24 + 1 , 14, 0}, Math.PI, false, true, false, 0.7, 0.3, 0.02);

        //Unlatch and push in
        hook1.setPosition(0.25);
        hook2.setPosition(0.25);
        apGoTo(new double[]{5*24 - 4 , 14, 0}, Math.PI/2, true, true, false, 0.7, 0.5, 0.015);

        //Park
        apGoTo(new double[]{3*24 , 28, 0}, Math.PI/2, true, true, true);

        while (opModeIsActive()) {
            autopilot.communicate(tracker);
            autopilot.telemetryUpdate();
            telemetry.update();
            idleStateMachines();
        }

         */

        /*
        while (opModeIsActive()) {
            autopilot.communicate(tracker);
            autopilot.telemetryUpdate();
            telemetry.update();
            idleStateMachines();
            telemetry.addData("x odometer:", myDrivetrain.getXOdometer().getCurrentPosition());
        }
        */





    public void idleStateMachines() {
        in.tick(intakeGo, false, intakeRetract);
        if (intakeGo) { intakeGo = false; }

        out.tick(triggerGrab, controlUp, controlDown, triggerRelease, armUp, armDown, autoPlace);
        if (triggerGrab) { triggerGrab = false; }
        if (triggerRelease) {triggerRelease = false; }

        side.tick(triggerSideGrab, triggerSideRelease);
        if (triggerSideGrab) {triggerSideGrab = false; }
        if (triggerSideRelease) {triggerSideRelease = false; }


    }
}
