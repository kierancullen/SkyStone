package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class GrabLiftPlaceController {

     int SAFE_REGION_LOWER= 225;
    int SAFE_REGION_UPPER = 235;
    int LIFT_SAFE_POSITION = 230 ;

    int DOWN_REGION_UPPER = 10;
    int MAX_HEIGHT = 1000;

    double WINCH_RESET_SPEED = 0.2;
    double WINCH_SAFE1_SPEED = 0.75;
    double WINCH_SAFE2_SPEED = 0.75;

    double SWING_WAITING_POSITION = 0.04; //0.16
    double TILT_WAITING_POSITION = 0; //0.05
    double GRAB_WAITING_POSITION= 0.17; //0.32

    double SWING_LOWERED_POSITION = 0.3; //0.26
    double TILT_LOWERED_POSITION = 0.21; //0.21

    double SWING_OUT_POSITION = 0.92;
    double TILT_LEVEL_POSITION = 0.85;

    double GRAB_CLOSED_POSITION = 0;

    long LOWER_GRIP_MS =200;
    long CLOSE_GRIP_MS =1000;
    long SWING1_MS = 1000;
    long SWING2_MS = 1000;
    long SWING3_MS =1000;
    long RELEASE_MS = 1000;

    int HEIGHT_TARGET_THRESHOLD = 10;
    int HEIGHT_STEP = 20;

    DcMotor winchLeft;
    DcMotor winchRight;

    Servo swing;
    Servo tilt;
    Servo grab;

    enum GLDState {
        RESET,
        WAIT_BRICK,
        TRANS,
        LOWER_GRIP,
        CLOSE_GRIP,
        WAIT_CLEAR1,
        SAFE1,
        SWING1,
        SWING2,
        USER_ADJUST,
        POST_ADJUST,
        OPEN_GRIP,
        WAIT_CLEAR2,
        SAFE2,
        SWING3,
    }


    GLDState currentState;
    GLDState lastState;
    long timeAtStateStart;
    int heightTarget;

    public GrabLiftPlaceController(DcMotor winchLeft, DcMotor winchRight, Servo swing, Servo tilt, Servo grab) {

        // Also ensure that winch directions get set correctly somewhere (here or in opmode class)
        // Positive direction is assumed to be up.
        winchLeft.setTargetPosition(0);
        winchRight.setTargetPosition(0);
        winchLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winchRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winchLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        winchRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.winchLeft = winchLeft;
        this.winchRight = winchRight;
        this.swing = swing;
        this.tilt = tilt;
        this.grab = grab;
    }

    public int getLiftPosition() {
        // Simple mean of encoder values
        return (winchLeft.getCurrentPosition() + winchRight.getCurrentPosition()) / 2;
    }

    public boolean liftIsInSafePosition() {
        // Returns true if the lift is at a height where the swing-arm can move in/out of the robot.
        int liftPosition = getLiftPosition();
        return liftPosition > SAFE_REGION_LOWER && liftPosition < SAFE_REGION_UPPER;
    }

    public boolean liftIsInDownPosition() {
        int liftPosition = getLiftPosition();
        return liftPosition < DOWN_REGION_UPPER;
    }

    public boolean liftIsAtBound() {
        int liftPosition = getLiftPosition();
        return liftPosition < DOWN_REGION_UPPER || liftPosition > MAX_HEIGHT;
    }

    public boolean liftIsAtHeightTarget() {
        int liftPosition = getLiftPosition();
        return Math.abs(liftPosition - heightTarget) < HEIGHT_TARGET_THRESHOLD;
    }


    public void start() {
        // Call once before calling calling tick. (In start() is the best place to do this)
        currentState = GLDState.RESET;
        lastState = GLDState.RESET;
        timeAtStateStart = System.currentTimeMillis();
    }

    public void tick(
            boolean triggerGrab, // Start sequence by grabbing brick inside robot
            boolean triggerClearBridge, // when it's OK to lift slightly and swing out
            boolean controlUp, // Variable speed for user control of lift up/down motion
            boolean controlDown,
            boolean triggerPlace, // Open grip
            boolean triggerClearTower, // when it's OK to zip the lift back down
            boolean revert
    ) {

        if (currentState == GLDState.RESET) {
            winchLeft.setTargetPosition(0);
            winchRight.setTargetPosition(0);
            winchLeft.setPower(WINCH_RESET_SPEED);
            winchRight.setPower(WINCH_RESET_SPEED);
            if (liftIsInDownPosition() && triggerClearBridge) {
                currentState = GLDState.WAIT_BRICK;
            }
        }

        else if (currentState == GLDState.WAIT_BRICK) {
            winchLeft.setPower(0);
            winchRight.setPower(0);
            swing.setPosition(SWING_WAITING_POSITION);
            tilt.setPosition(TILT_WAITING_POSITION);
            grab.setPosition(GRAB_WAITING_POSITION);
            if (triggerGrab) {
                currentState = GLDState.TRANS;
            }
        }

        else if (currentState == GLDState.TRANS) {
            swing.setPosition(0.08);
            grab.setPosition(0.15);
            if (System.currentTimeMillis() - timeAtStateStart > LOWER_GRIP_MS) {
                currentState = GLDState.LOWER_GRIP;
            }
        }

        else if (currentState == GLDState.LOWER_GRIP) {
            swing.setPosition(SWING_LOWERED_POSITION);
            tilt.setPosition(TILT_LOWERED_POSITION);
            if (System.currentTimeMillis() - timeAtStateStart > LOWER_GRIP_MS) {
                currentState = GLDState.CLOSE_GRIP;
            }
        }

        else if (currentState == GLDState.CLOSE_GRIP) {
            grab.setPosition(GRAB_CLOSED_POSITION);
            if (System.currentTimeMillis() - timeAtStateStart > CLOSE_GRIP_MS) {
                currentState = GLDState.WAIT_CLEAR1;
            }
        }

        else if (currentState == GLDState.WAIT_CLEAR1) {
            if (triggerClearBridge) {
                currentState = GLDState.SAFE1;
            }
            else if (revert) {
                currentState = GLDState.TRANS;
            }
        }

        else if (currentState == GLDState.SAFE1) {
            winchLeft.setTargetPosition(LIFT_SAFE_POSITION);
            winchRight.setTargetPosition(LIFT_SAFE_POSITION);
            winchLeft.setPower(WINCH_SAFE1_SPEED);
            winchRight.setPower(WINCH_SAFE1_SPEED);
            if (liftIsInSafePosition()) {
                currentState = GLDState.SWING1;
            }
        }

        else if (currentState == GLDState.SWING1) {
            tilt.setPosition(TILT_LEVEL_POSITION);
            swing.setPosition(SWING_OUT_POSITION);
            if (System.currentTimeMillis() - timeAtStateStart > SWING1_MS) {
                currentState = GLDState.USER_ADJUST;
                heightTarget = LIFT_SAFE_POSITION;
            }
        }

        else if (currentState == GLDState.SWING2) {
            tilt.setPosition(TILT_LEVEL_POSITION);
            if (System.currentTimeMillis() - timeAtStateStart > SWING2_MS) {
                currentState = GLDState.USER_ADJUST;
                heightTarget = LIFT_SAFE_POSITION;

            }
        }

        else if (currentState == GLDState.USER_ADJUST) {

            if (liftIsAtHeightTarget() && !liftIsAtBound()) { // Allow movement
                if (controlUp) {
                    heightTarget += HEIGHT_STEP;
                }
                else if (controlDown) {
                    heightTarget -= HEIGHT_STEP;
                }
            }
            if (heightTarget > MAX_HEIGHT) {
                heightTarget = MAX_HEIGHT;
            }
            if (heightTarget < DOWN_REGION_UPPER) {
                heightTarget = 0;
            }
            winchLeft.setTargetPosition(heightTarget);
            winchRight.setTargetPosition(heightTarget);
            winchLeft.setPower(1);
            winchRight.setPower(1);

                if (triggerPlace) {
                    currentState= GLDState.OPEN_GRIP;
                }
        }



        else if (currentState == GLDState.OPEN_GRIP) {
                grab.setPosition(GRAB_WAITING_POSITION);
                if (System.currentTimeMillis() - timeAtStateStart > RELEASE_MS) {
                    currentState = GLDState.POST_ADJUST; //WAIT_CLEAR2

                }
            }

        else if (currentState == GLDState.POST_ADJUST) {

            if (liftIsAtHeightTarget() && !liftIsAtBound()) { // Allow movement
                if (controlUp) {
                    heightTarget += HEIGHT_STEP;
                }
                else if (controlDown) {
                    heightTarget -= HEIGHT_STEP;
                }
            }
            if (heightTarget > MAX_HEIGHT) {
                heightTarget = MAX_HEIGHT;
            }
            if (heightTarget < DOWN_REGION_UPPER) {
                heightTarget = 0;
            }
            winchLeft.setTargetPosition(heightTarget);
            winchRight.setTargetPosition(heightTarget);
            winchLeft.setPower(1);
            winchRight.setPower(1);

            if (triggerClearTower) {
                currentState= GLDState.WAIT_CLEAR2;
            }
        }

        else if (currentState == GLDState.WAIT_CLEAR2) {
                if (triggerClearTower) {
                    currentState = GLDState.SAFE2;
                }
        }

        else if (currentState == GLDState.SAFE2) {
            winchLeft.setTargetPosition(LIFT_SAFE_POSITION);
            winchRight.setTargetPosition(LIFT_SAFE_POSITION);
            winchLeft.setPower(WINCH_SAFE2_SPEED);
            winchRight.setPower(WINCH_SAFE2_SPEED);
            if (liftIsInSafePosition()) {
                currentState = GLDState.SWING3;
            }
        }

        else if (currentState == GLDState.SWING3) {
            // As if we just grabbed the brick (avoid hitting front cross)
            // Once lift is all the way down, WAIT_BRICK sets it back to waiting positions
            swing.setPosition(SWING_LOWERED_POSITION);
            tilt.setPosition(TILT_LOWERED_POSITION);
            grab.setPosition(GRAB_CLOSED_POSITION);
            if (System.currentTimeMillis() - timeAtStateStart > SWING3_MS) {
                currentState = GLDState.RESET;
            }
        }



        if (currentState != lastState) {
            // There was a state transition
            timeAtStateStart = System.currentTimeMillis();
        }
        lastState = currentState;
    }

}