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
import static org.firstinspires.ftc.teamcode.GlobalMovement.movement_turn;
import static org.firstinspires.ftc.teamcode.GlobalMovement.movement_x;
import static org.firstinspires.ftc.teamcode.GlobalMovement.movement_y;

public class AutoCommonFaster extends LinearOpMode {
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

    //Robot start positions
    public static double[] ROBOT_INIT_POSITION = new double[]{36, 9.2, 0};
    public static double[] ROBOT_INIT_ATTITUDE = new double[]{0, 0, 0};
    public static double[] INVERT_ROBOT_INIT_POSITION = new double[]{-36, 9.2, 0};
    public static double[] INVERT_ROBOT_INIT_ATTITUDE = new double[]{0, 0, 0};
    
    //Odometry constants, based on the physical locations of our wheels
    public static double DUALODO_X_RADIUS = 3.25614173 - 2.3134252;
    public static double DUALODO_Y_RADIUS = -0.25051181102 + 7.184370097;
    public static double DUALODO_TICKS_PER_UNIT = 1440 /  (1.88976 * Math.PI);

    
    
    public static int AP_COUNTS_TO_STABLE = 10; //The number of loop iterations that need to pass with the robot within AP_NAV_UNITS_TO_STABLE of the target before we can move on
    public static double AP_NAV_UNITS_TO_STABLE = 1; // inch, Distance to the target position before we go to the next move
    public static double AP_ORIENT_UNITS_TO_STABLE = 0.05; // rad, Difference between current angle and desired angle before we go to the next move


    //Used for 270deg servos 
    public void setServoExtendedRange(Servo servo, int min, int max) {
        ServoControllerEx controller = (ServoControllerEx) servo.getController();
        int servoPort = servo.getPortNumber();
        PwmControl.PwmRange range = new PwmControl.PwmRange(min, max);
        controller.setServoPwmRange(servoPort, range);
    }

    //Method called to navigate to a point
    public void apGoTo(double[] pos, double hdg, boolean useOrientation, boolean useTranslation, boolean fullStop) {
        AutopilotSegment seg = new AutopilotSegment();
        seg.navigationTarget = pos;
        seg.orientationTarget = hdg;
        seg.navigationGain = 0.015; //Gain for translation, lower gain
        seg.orientationGain = 1.25; //Gain for rotation
        seg.navigationMax = 1.0; //Maximum translation power
        seg.navigationMin = 0.25; //Minimum translation power
        seg.orientationMax = 0.5; //Maximum rotation power 
        seg.useOrientation = useOrientation; //Set this to false to do a move that only cares about robot position and not angle
        seg.useTranslation = useTranslation; //Set this to false to do a turn
        
        //Fullstop controls whether autopilot tries to stop the robot completely at a certain point, or continues to the next point.
        //On a path consisting of multiple points that we want to zoom through, fullstop should be false
        seg.fullStop = fullStop; 

        autopilot.setNavigationTarget(seg);
        autopilot.setNavigationStatus(AutopilotHost.NavigationStatus.RUNNING);

        double [] yxh = null;
        long lastTime = System.currentTimeMillis();

        //autopilot.getNavigationStatus switches to STOPPED once we've determined that the target point is reached, and then this loop ends
        while (autopilot.getNavigationStatus() == AutopilotHost.NavigationStatus.RUNNING && opModeIsActive()) { 

            idleStateMachines(); 
            
            //yxh is returned by autopilot after each iteration of this loop
            //It is essentially a vector for x and y translation, as well as rotation
            //In our setup, we put these values into global variables and then tell the drivetrain to update motor powers based on them
            if (yxh != null) {
                /*GlobalMovement.*/movement_y = yxh[0];
                /*GlobalMovement.*/movement_x = yxh[1];
                /*GlobalMovement.*/movement_turn = yxh[2];
                myDrivetrain.updatePowers();
            }
            autopilot.communicate(tracker); //Updates the robot position based on odometry tracking

            long timeNow = System.currentTimeMillis();
            telemetry.addData("FPS", 1000.0 / (timeNow - lastTime)); //Tracking loop iteration time
            lastTime = timeNow;

            //AutopilotSystem.visualizerBroadcastRoutine(autopilot);
            autopilot.telemetryUpdate();
            telemetry.update();

            yxh = autopilot.navigationTick(); //Get the new vector for robot translation and rotation
        }

    }
    
    //Some similar apGoTo methods that allow more parameters to be set 
    
    public void apGoTo(double[] pos, double hdg, boolean useOrientation, boolean useTranslation, boolean fullStop, double navigationMax, double navigationMin, double navigationGain) {
        apGoTo(pos, hdg, useOrientation, useTranslation, fullStop, navigationMax, navigationMin, navigationGain, AP_NAV_UNITS_TO_STABLE);
    }

    public void apGoTo(double[] pos, double hdg, boolean useOrientation, boolean useTranslation, boolean fullStop, double navigationMax, double navigationMin, double navigationGain, double navigationUnitsToStable) {
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

    //Legacy method for driving differential with no strafing, can also set seg.diffmode = true instead 
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

        
        //Instantiate autopilot, and create the tracker
        autopilot = new AutopilotHost(telemetry);
        tracker = new AutopilotTrackerTripleOdo(
                myDrivetrain.getXOdometer(),
                myDrivetrain.getYOdometerLeft(),
                myDrivetrain.getYOdometerRight(),
                DUALODO_X_RADIUS, DUALODO_Y_RADIUS,
                DUALODO_TICKS_PER_UNIT
        );

        
        ((AutopilotTrackerTripleOdo) tracker).setInverts(false, true, false); //Controls whether we need to invert the direction of our odometry encoders
        autopilot.setCountsToStable(AP_COUNTS_TO_STABLE);
        autopilot.setNavigationUnitsToStable(AP_NAV_UNITS_TO_STABLE);
        autopilot.setOrientationUnitsToStable(AP_ORIENT_UNITS_TO_STABLE);

        //The invert boolean allows all cooredinates to be reversed for the opposite side of the field
        if (invert) {
            autopilot.setNavigationTargetInverts(new boolean[]{true, false, false});
            autopilot.setOrientationTargetInvert(true);
        }

        // record any drift that happened while waiting in init, and zero it out
        autopilot.communicate(tracker);
        if (!invert) {
            tracker.setRobotAttitude(ROBOT_INIT_ATTITUDE);
            tracker.setRobotPosition(ROBOT_INIT_POSITION);
        }
        else {
            tracker.setRobotAttitude(INVERT_ROBOT_INIT_ATTITUDE);
            tracker.setRobotPosition(INVERT_ROBOT_INIT_POSITION);
        }

        //Scan the quarry to find the skystone locations
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

        //Start intake and outtake state machines
        in.start();
        out.start();



        //Beginning of actual auto
        int location = popper.locations[1];
        
        //Collect the first stone
        if (location == 0) {
            apGoTo(new double[]{20, 32, 0}, Math.PI/6, true, true, false);
            apGoTo(new double[]{18, 37, 0}, Math.PI/6, true, true, false);
            intakeGo = true;
            apGoTo(new double[]{15, 32, 0}, 0, true, true, false);
        }
        if (location == 1) {
            apGoTo(new double[]{20, 36, 0}, Math.PI/6, true, true, false);
            apGoTo(new double[]{16, 40, 0}, Math.PI/6, true, true, false);
            intakeGo = true;
            apGoTo(new double[]{20, 36, 0}, Math.PI/2, true, true, false);
            triggerGrab = true;
        }
        if (location == 2) {
            apGoTo(new double[]{28, 36, 0}, Math.PI/6, true, true, false);
            apGoTo(new double[]{26, 40, 0}, Math.PI/6, true, true, false);
            intakeGo = true;
            apGoTo(new double[]{28, 36, 0}, Math.PI/2, true, true, false);
            triggerGrab = true;
        }
        if (location == 3) {
            apGoTo(new double[]{36, 36, 0}, Math.PI/6, true, true, false);
            apGoTo(new double[]{34, 40, 0}, Math.PI/6, true, true, false);
            intakeGo = true;
            apGoTo(new double[]{36, 36, 0}, Math.PI/2, true, true, false);
            triggerGrab = true;

        }
        if (location == 4) {
            apGoTo(new double[]{44, 36, 0}, Math.PI/6, true, true, false);
            apGoTo(new double[]{42, 40, 0}, Math.PI/6, true, true, false);
            intakeGo = true;
            apGoTo(new double[]{44, 36, 0}, Math.PI/2, true, true, false);
            triggerGrab = true;
        }
        if (location == 5) {
            apGoTo(new double[]{52, 36, 0}, Math.PI/6, true, true, false);
            apGoTo(new double[]{50, 40, 0}, Math.PI/6, true, true, false);
            intakeGo = true;
            apGoTo(new double[]{52, 36, 0}, Math.PI/2, true, true, false);
            triggerGrab = true;
        }

        //Drive under the skybridge 
        apGoTo(new double[]{5*24 , 42, 0}, Math.PI/2, true, true, false, 0.7, 0.2, 0.03, 6); //28
        autoPlace=true;
        //Drive against the foundation, grab it, and release the stone
        apGoTo(new double[]{5*24 , 44, 0}, Math.PI, true, true, true, 0.5, 0.3, 0.02, 1); //38
        grab1.setPosition(0.53);
        grab2.setPosition(0.53);
        triggerRelease=true;
        sleep(1000);
        //Pull the foundation out
        apGoToNoStrafe(new double[]{4*24 + 1 , 20, 0}, Math.PI, false, true, false, 0.7, 0.3, 0.02);
        apGoTo(new double[]{4*24 +1 , 20, 0}, Math.PI/2, true, false, false, 0.7, 0.3, 0.02);
        grab1.setPosition(0.25);
        grab2.setPosition(0.25);
        //Push the foundation into the wall
        apGoTo(new double[]{4*24 - 5 , 24, 0}, Math.PI/2, true, true, false, 1.0, 1.0, 0.05, 3);
        apGoTo(new double[]{5*24 - 4 , 24, 0}, Math.PI/2, true, true, false, 0.7, 0.7, 0.03, 2);
        
        //Drive back under the skybirdge 
        apGoTo(new double[]{3*24 , 32, 0}, Math.PI/2, true, true, false, 0.7, 0.7, 0.03, 3);


        autoPlace = false;
        location =  popper.locations[0];
        
        //Get the second stone 
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
        
        //Go back under the bridge 
        apGoTo(new double[]{5*24 - 18 , 36, 0}, Math.PI/2, true, true, false, 0.7, 0.5, 0.03, 5);
        autoPlace=true;
        //Bump up against the foundation and place 
        apGoTo(new double[]{5*24 - 9 , 36, 0}, Math.PI/2, true, true, true, 0.7, 0.5, 0.03, 5);
        sleep(2000);
        triggerRelease=true;
        while(out.currentState != OuttakeController2.OuttakeState.RELEASING) idleStateMachines();
        sleep(1000);
        
        //Park, set motor powers to zero 
        apGoTo(new double[]{3*24 , 36, 0}, Math.PI/2, true, true, true, 0.7, 0.5, 0.03, 3); //28
        tl.setZeroPowerBehavior(BRAKE);
        tr.setZeroPowerBehavior(BRAKE);
        br.setZeroPowerBehavior(BRAKE);
        bl.setZeroPowerBehavior(BRAKE);

        tr.setPower(0);
        tl.setPower(0);
        bl.setPower(0);
        br.setPower(0);




        
        while (opModeIsActive()) {
            autopilot.communicate(tracker);
            autopilot.telemetryUpdate();
            telemetry.update();
            idleStateMachines();
        }
    }

    //Controls state machines for intake and outtake 
    public void idleStateMachines() {
        in.tick(intakeGo, false, false);
        if (intakeGo) { intakeGo = false; }
        out.tick(triggerGrab, controlUp, controlDown, triggerRelease, armUp, armDown, autoPlace, false);
        if (triggerGrab) { triggerGrab = false; }
        if (triggerRelease) {triggerRelease = false; }
        if (autoPlace) {autoPlace = false;}
    }
}
