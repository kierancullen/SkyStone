package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeController {

    double GRAB_GRABBING_POSITION;
    double GRAB_OPEN_POSITION;

    int LIFT_LOWER_BOUND;
    int LIFT_UPPER_BOUND;
    int RELEASE_LIFT; // extra height on release

    double SLIDE_EXTENDING_POWER;
    double SLIDE_RETRACTING_POWER; // negative

    double HUMAN_UP_POWER;

    long GRABBING_MS;
    long EXTENDING_MS;
    long RELEASING_MS;
    long RETRACTING_MS;


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
    }

    OuttakeState currentState;
    OuttakeState lastState;
    long timeAtStateStart;

    public OuttakeController(DcMotor winchLeft, DcMotor winchRight, CRServo slide, Servo grab) {

        // Also ensure that winch directions get set correctly somewhere (here or in opmode class)
        // Positive direction is assumed to be up.
        winchLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winchRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winchLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        winchRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        winchLeft.setPower(0);
        winchRight.setPower(0);
        grab.setPosition(GRAB_GRABBING_POSITION);

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
            winchLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            winchRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
            if (controlUp) { // up
                winchLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                winchRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                winchLeft.setPower(HUMAN_UP_POWER);
                winchRight.setPower(HUMAN_UP_POWER);
            }
            else if (controlDown) { // fall
                winchLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                winchRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                winchLeft.setPower(0);
                winchRight.setPower(0);
            }
            else if (triggerRelease) {
                currentState = OuttakeState.RELEASING;
            }
            else { // stay put
                if (winchLeft.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                    winchLeft.setTargetPosition(getLiftPosition());
                    winchRight.setTargetPosition(getLiftPosition());
                    winchLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    winchRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    winchLeft.setPower(1);
                    winchRight.setPower(1);
                }
            }
        }

        else if (currentState == OuttakeState.RELEASING) {
            grab.setPosition(GRAB_OPEN_POSITION);
            winchLeft.setTargetPosition(getLiftPosition() + RELEASE_LIFT);
            winchRight.setTargetPosition(getLiftPosition() + RELEASE_LIFT);
            winchLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            winchRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            winchLeft.setPower(1);
            winchRight.setPower(1);
            if (System.currentTimeMillis() - timeAtStateStart > RELEASING_MS) {
                currentState = OuttakeState.RETRACTING;
            }
        }

        else if (currentState == OuttakeState.RETRACTING) {
            slide.setPower(SLIDE_RETRACTING_POWER);
            if (System.currentTimeMillis() - timeAtStateStart > RETRACTING_MS) {
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
