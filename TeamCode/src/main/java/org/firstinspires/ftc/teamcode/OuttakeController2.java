package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeController2 {

    boolean set;

    double GRAB_GRABBING_POSITION = 0.71;
    double GRAB_OPEN_POSITION = 0.46;

    int LIFT_LOWER_BOUND = 15;
    int LIFT_UPPER_BOUND = 1000;
    int RELEASE_LIFT = 200; // extra height on release

    double PRIME_POS = 0.1;
    double GRIP_POS = 0.01;
    double armTravel = 0.0;
    double RELEASE_POS = 0.3;
    double HUMAN_UP_POWER = 0.7;
    double HUMAN_DOWN_POWER = 0;
    double FALLING_DOWN_POWER = -0.1;

    long GRABBING_MS = 500;
    long EXTENDING_MS = 400;
    long RELEASING_MS = 1000;
    long RETRACTING_MS = 400;




    DcMotor winchLeft;
    DcMotor winchRight;
    Servo arm1;
    Servo arm2;


    Servo grip;

    //CRServo slide;
    //Servo grab;

    enum OuttakeState {
        READY,
        DOWN,
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

    public OuttakeController2 (DcMotor winchLeft, DcMotor winchRight, Servo arm1, Servo arm2, Servo grip) {

        // Also ensure that winch directions get set correctly somewhere (here or in opmode class)
        // Positive direction is assumed to be up.
        winchLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        winchRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        winchLeft.setPower(0);
        winchRight.setPower(0);

        this.arm1 = arm1;
        this.arm2 = arm2;
        this.grip  = grip;

        this.winchLeft = winchLeft;
        this.winchRight = winchRight;
        //this.slide = slide;
        //this.grab = grab;
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
            boolean triggerRelease,
            boolean armUp,
            boolean armDown,
            boolean autoPlace
    ) {

        if (currentState == OuttakeState.READY) {
            winchLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            winchRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            grip.setPosition(GRAB_OPEN_POSITION);
            //slide.setPower(0);
            arm1.setPosition(PRIME_POS);
            arm2.setPosition(PRIME_POS);
            winchLeft.setPower(0);
            winchRight.setPower(0);
            if (triggerGrab) {
                currentState = OuttakeState.DOWN;
            }
        }

        else if (currentState == OuttakeState.DOWN) {
            arm1.setPosition(GRIP_POS);
            arm2.setPosition(GRIP_POS);
            if (System.currentTimeMillis() - timeAtStateStart > GRABBING_MS) {
                currentState = OuttakeState.GRABBING;
            }
        }

        else if (currentState == OuttakeState.GRABBING) {

            grip.setPosition(GRAB_GRABBING_POSITION);
            if (System.currentTimeMillis() - timeAtStateStart > GRABBING_MS) {
                currentState = OuttakeState.HUMAN;
                arm1.setPosition(PRIME_POS);
                arm2.setPosition(PRIME_POS);
            }
        }



        /*else if (currentState == OuttakeState.EXTENDING) {
            //slide.setPower(SLIDE_EXTENDING_POWER);
            if (System.currentTimeMillis() - timeAtStateStart > EXTENDING_MS) {
                currentState = OuttakeState.HUMAN;
            }
        } */

        else if (currentState == OuttakeState.HUMAN) {
            //slide.setPower(0);
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
            else if (triggerRelease && System.currentTimeMillis() - timeAtStateStart > 1000) { //Ensure that auto doesn't go through this state too fast
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

                if (armUp) {
                   armTravel += 0.01;

                }
                if (armDown) {
                    armTravel -= 0.01;
                }
                if (autoPlace) {
                    armTravel = 0.7;
                }

                arm1.setPosition(PRIME_POS + armTravel);
                arm2.setPosition(PRIME_POS + armTravel);

            }
        }

        else if (currentState == OuttakeState.RELEASING) {
            armTravel = 0.0;
            grip.setPosition(GRAB_OPEN_POSITION);
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
            //grab.setPosition(GRAB_GRABBING_POSITION);
            //slide.setPower(SLIDE_RETRACTING_POWER);
            arm1.setPosition(RELEASE_POS);
            arm2.setPosition(RELEASE_POS);
            if (System.currentTimeMillis() - timeAtStateStart > RETRACTING_MS) {
                currentState = OuttakeState.FALLING;
            }
        }

        else if (currentState == OuttakeState.FALLING) {
            winchLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            winchRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            //grab.setPosition(GRAB_OPEN_POSITION);
            winchLeft.setPower(FALLING_DOWN_POWER);
            winchRight.setPower(FALLING_DOWN_POWER);
            winchLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            winchRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            winchLeft.setPower(FALLING_DOWN_POWER);
            winchRight.setPower(FALLING_DOWN_POWER);
            //slide.setPower(0);
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
