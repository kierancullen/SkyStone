package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeController2 {

    boolean readyForGrab;

    double GRAB_GRABBING_POSITION = 0.71;
    double GRAB_OPEN_POSITION = 0.06;

    int LIFT_LOWER_BOUND = 15;
    int LIFT_UPPER_BOUND = 1150;
    int RELEASE_LIFT = 100; // extra height on release

    double PRIME_POS = 0.8;
    double GRIP_POS = 0.96;
    double armTravel = 0.0;
    double RELEASE_POS = 0.3;
    double HUMAN_UP_POWER = 1.0;
    double HUMAN_DOWN_POWER = -0.1;
    double FALLING_DOWN_POWER = -0.1;

    long GRABBING_MS = 500;
    long EXTENDING_MS = 400;
    long RELEASING_MS = 700;
    long RETRACTING_MS = 400;

    int placingLevel;




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
        LIFTING,
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

    public int levelLiftPosition() {
        if (placingLevel <= 3) {
            return ((placingLevel - 1) * 120);
        }
        else {
            return ((placingLevel - 3) * 120);
        }
    }

    public double levelArmPosition() {
        if (placingLevel == 1) return 0;
        else if (placingLevel == 2) return 0;
        else if (placingLevel == 3) return 0;
        else return 0.3;
    }


    public void start() {
        // Call once before calling calling tick. (In start() is the best place to do this)
        currentState = OuttakeState.READY;
        lastState = OuttakeState.READY;
        timeAtStateStart = System.currentTimeMillis();

        placingLevel = 1;
    }

    public void tick(
            boolean triggerGrab, // Start sequence by grabbing brick inside robot
            boolean controlUp,
            boolean controlDown,
            boolean triggerRelease,
            boolean armUp,
            boolean armDown,
            boolean autoPlace,
            boolean intakeBlock,
            boolean levelUp,
            boolean levelDown
    ) {
        if (levelUp) placingLevel++;
        if (levelDown) placingLevel--;

        if (currentState == OuttakeState.READY) {
            winchLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            winchRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            grip.setPosition(GRAB_OPEN_POSITION);
            //slide.setPower(0);
            if (intakeBlock) {
                arm1.setPosition(0);
                arm2.setPosition(0);
            }
            else {
                arm1.setPosition(PRIME_POS);
                arm2.setPosition(PRIME_POS);
            }

            winchLeft.setPower(0);
            winchRight.setPower(0);
            if (triggerGrab || readyForGrab) {
                currentState = OuttakeState.DOWN;
            }
        }

        else if (currentState == OuttakeState.DOWN) {
            arm1.setPosition(GRIP_POS);
            arm2.setPosition(GRIP_POS);
            if (triggerGrab) {
                currentState = OuttakeState.GRABBING;
            }
        }

        else if (currentState == OuttakeState.GRABBING) {

            grip.setPosition(GRAB_GRABBING_POSITION);
            if (triggerGrab && System.currentTimeMillis() - timeAtStateStart > GRABBING_MS) {
                currentState = OuttakeState.LIFTING;
                arm1.setPosition(PRIME_POS);
                arm2.setPosition(PRIME_POS);
            }
        }

        else if (currentState == OuttakeState.LIFTING) {
            winchLeft.setTargetPosition(levelLiftPosition());
            winchRight.setTargetPosition(levelLiftPosition());
            winchLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            winchRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            winchLeft.setPower(1);
            winchRight.setPower(1);
            grip.setPosition(GRAB_GRABBING_POSITION);
            if (Math.abs(getLiftPosition() - levelLiftPosition()) < 10) {
                currentState = OuttakeState.HUMAN;
            }
        }


        else if (currentState == OuttakeState.HUMAN) {
            winchLeft.setTargetPosition(getLiftPosition());
            winchRight.setTargetPosition(getLiftPosition());
            if (armUp) {
                armTravel += 0.1;

            }
            if (armDown) {
                armTravel -= 0.1;
            }
            if (autoPlace) {
                armTravel = 0.7;
            }
            arm1.setPosition(levelArmPosition() + armTravel);
            arm2.setPosition(levelArmPosition() + armTravel);

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
            else if (triggerGrab && System.currentTimeMillis() - timeAtStateStart > 1000) {
                currentState = OuttakeState.RELEASING;
                placingLevel++;
            }
            else { // stay put
                if (winchLeft.getMode() == DcMotor.RunMode.RUN_USING_ENCODER || winchLeft.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                    winchLeft.setTargetPosition(winchLeft.getCurrentPosition());
                    winchRight.setTargetPosition(winchRight.getCurrentPosition());
                    winchLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    winchRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    winchLeft.setPower(1);
                    winchRight.setPower(1);
                }


            }
        }

        else if (currentState == OuttakeState.RELEASING) {

            armTravel = 0.0;
            grip.setPosition(GRAB_OPEN_POSITION);
            winchLeft.setTargetPosition(winchLeft.getCurrentPosition() + RELEASE_LIFT);
            winchRight.setTargetPosition(winchRight.getCurrentPosition() + RELEASE_LIFT);
            winchLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            winchRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            winchLeft.setPower(1);
            winchRight.setPower(1);

            if (System.currentTimeMillis() - timeAtStateStart > RELEASING_MS) {
                currentState = OuttakeState.RETRACTING;
            }
        }

        else if (currentState == OuttakeState.RETRACTING) {
            arm1.setPosition(PRIME_POS);
            arm2.setPosition(PRIME_POS);
            if (System.currentTimeMillis() - timeAtStateStart > RETRACTING_MS) {
                currentState = OuttakeState.FALLING;
            }
        }

        else if (currentState == OuttakeState.FALLING) {
            winchLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            winchRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            winchLeft.setPower(FALLING_DOWN_POWER);
            winchRight.setPower(FALLING_DOWN_POWER);
            winchLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            winchRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            winchLeft.setPower(FALLING_DOWN_POWER);
            winchRight.setPower(FALLING_DOWN_POWER);
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
