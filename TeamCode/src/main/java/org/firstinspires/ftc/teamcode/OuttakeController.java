package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeController {

    boolean set;

    double GRAB_GRABBING_POSITION = 0.7;
    double GRAB_OPEN_POSITION = 0;

    int LIFT_LOWER_BOUND = 15;
    int LIFT_UPPER_BOUND = 1000;
    int RELEASE_LIFT = 100; // extra height on release

    double SLIDE_EXTENDING_POWER = 0.5;
    double SLIDE_RETRACTING_POWER = -0.5; // negative

    double HUMAN_UP_POWER = 0.50;
    double HUMAN_DOWN_POWER = 0;
    double FALLING_DOWN_POWER = -0.1;

    long GRABBING_MS = 500;
    long EXTENDING_MS = 400;
    long RELEASING_MS = 1000;
    long RETRACTING_MS = 400;


    DcMotor winchLeft;
    DcMotor winchRight;

    CRServo slide;
    Servo grab;

    enum OuttakeState {
        READY,
        GRABBING,
        EXTENDING,
        HUMAN,
        RELEASING,
        RETRACTING,
        FALLING,
    }

    OuttakeState currentState;
    OuttakeState lastState;
    long timeAtStateStart;

    public OuttakeController(DcMotor winchLeft, DcMotor winchRight, CRServo slide, Servo grab) {

        // Also ensure that winch directions get set correctly somewhere (here or in opmode class)
        // Positive direction is assumed to be up.
        winchLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        winchRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        winchLeft.setPower(0);
        winchRight.setPower(0);

        this.winchLeft = winchLeft;
        this.winchRight = winchRight;
        this.slide = slide;
        this.grab = grab;
    }

    public int getLiftPosition() {
        // Simple mean of encoder values
        return (winchLeft.getCurrentPosition() + winchRight.getCurrentPosition()) / 2;
    }

    public boolean liftCanGoDown() {
        int liftPosition = getLiftPosition();
        return liftPosition > LIFT_LOWER_BOUND;
    }

    public boolean liftCanGoUp() {
        int liftPosition = getLiftPosition();
        return liftPosition < LIFT_UPPER_BOUND;
    }

    public boolean liftIsNearBottom() {
        int liftPosition = getLiftPosition();
        return liftPosition < 20;
    }


    public void start() {
        // Call once before calling calling tick. (In start() is the best place to do this)
        currentState = OuttakeState.READY;
        lastState = OuttakeState.READY;
        timeAtStateStart = System.currentTimeMillis();
    }

    public void tick(
            boolean triggerGrab, // Start sequence by grabbing brick inside robot
            boolean controlUp,
            boolean controlDown,
            boolean triggerRelease
    ) {

        if (currentState == OuttakeState.READY) {
            winchLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            winchRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            grab.setPosition(GRAB_OPEN_POSITION);
            slide.setPower(0);
            winchLeft.setPower(0);
            winchRight.setPower(0);
            if (triggerGrab) {
                currentState = OuttakeState.GRABBING;
            }
        }

        else if (currentState == OuttakeState.GRABBING) {
            grab.setPosition(GRAB_GRABBING_POSITION);
            if (System.currentTimeMillis() - timeAtStateStart > GRABBING_MS) {
                currentState = OuttakeState.EXTENDING;
            }
        }

        else if (currentState == OuttakeState.EXTENDING) {
            slide.setPower(SLIDE_EXTENDING_POWER);
            if (System.currentTimeMillis() - timeAtStateStart > EXTENDING_MS) {
                currentState = OuttakeState.HUMAN;
            }
        }

        else if (currentState == OuttakeState.HUMAN) {
            slide.setPower(0);
            if (controlUp && liftCanGoUp()) {
                winchLeft.setPower(HUMAN_UP_POWER);
                winchRight.setPower(HUMAN_UP_POWER);
                if (winchLeft.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                    winchLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    winchRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                winchLeft.setPower(HUMAN_UP_POWER);
                winchRight.setPower(HUMAN_UP_POWER);
            }
            else if (controlDown && liftCanGoDown()) {
                winchLeft.setPower(HUMAN_DOWN_POWER);
                winchRight.setPower(HUMAN_DOWN_POWER);
                if (winchLeft.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                    winchLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    winchRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                winchLeft.setPower(HUMAN_DOWN_POWER);
                winchRight.setPower(HUMAN_DOWN_POWER);
            }
            else if (triggerRelease) {
                currentState = OuttakeState.RELEASING;
                set = false;
            }
            else { // stay put
                if (winchLeft.getMode() == DcMotor.RunMode.RUN_USING_ENCODER || winchLeft.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                    winchLeft.setTargetPosition(winchLeft.getCurrentPosition() + 10);
                    winchRight.setTargetPosition(winchRight.getCurrentPosition() + 10);
                    winchLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    winchRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    winchLeft.setPower(1);
                    winchRight.setPower(1);
                }
            }
        }

        else if (currentState == OuttakeState.RELEASING) {
            grab.setPosition(GRAB_OPEN_POSITION);
            if (!set) {
                winchLeft.setTargetPosition(winchLeft.getCurrentPosition() + RELEASE_LIFT);
                winchRight.setTargetPosition(winchRight.getCurrentPosition() + RELEASE_LIFT);
                winchLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                winchRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                winchLeft.setPower(1);
                winchRight.setPower(1);
                set = true;
            }
            if (System.currentTimeMillis() - timeAtStateStart > RELEASING_MS) {
                currentState = OuttakeState.RETRACTING;
            }
        }

        else if (currentState == OuttakeState.RETRACTING) {
            grab.setPosition(GRAB_GRABBING_POSITION);
            slide.setPower(SLIDE_RETRACTING_POWER);
            if (System.currentTimeMillis() - timeAtStateStart > RETRACTING_MS) {
                currentState = OuttakeState.FALLING;
            }
        }

        else if (currentState == OuttakeState.FALLING) {
            winchLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            winchRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            grab.setPosition(GRAB_OPEN_POSITION);
            winchLeft.setPower(FALLING_DOWN_POWER);
            winchRight.setPower(FALLING_DOWN_POWER);
            winchLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            winchRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            winchLeft.setPower(FALLING_DOWN_POWER);
            winchRight.setPower(FALLING_DOWN_POWER);
            slide.setPower(0);
            /*if (liftIsNearBottom()) {
                winchLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                winchRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }*/
            if (!liftCanGoDown()) {
                currentState = OuttakeState.READY;
            }
        }


        if (currentState != lastState) {
            // There was a state transition
            timeAtStateStart = System.currentTimeMillis();
        }
        lastState = currentState;
    }

}
