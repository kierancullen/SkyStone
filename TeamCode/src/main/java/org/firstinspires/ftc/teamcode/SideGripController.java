package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class SideGripController {

    boolean invert = false;

    double GRIP_OPEN_POS = 0.158;
    double GRIP_CLOSED_POS = 0;
    double SWING_IN_POS = 0;
    double SWING_DOWN_POS = 0.32;
    double SWING_RELEASE_POS = 0.15;

    double SWING_DOWN_MS = 1000;
    double GRIP_MS = 1000;
    double SWING_UP_MS = 1000;
    double RELEASING_MS = 1000;

    Servo redGrip;
    Servo redGripSwing;
    Servo blueGrip;
    Servo blueGripSwing;

    boolean safeToMove = false;

    enum SideGripState {
        RETRACTED,
        READY,
        GRIPPING,
        CARRYING,
        LIFTING,
        RELEASING
    }

    SideGripState currentState;
    SideGripState lastState;
    long timeAtStateStart;

    OuttakeController2 outtake;

    public SideGripController(Servo redGrip, Servo redGripSwing, Servo blueGrip, Servo blueGripSwing, boolean invert) {

        this.redGrip = redGrip;
        this.redGripSwing = redGripSwing;
        this.blueGrip = redGrip;
        this.blueGripSwing = redGripSwing;
        this.invert = invert;

    }


    public void start() {
        // Call once before calling calling tick. (In start() is the best place to do this)
        currentState = SideGripState.RETRACTED;
        lastState = SideGripState.RETRACTED;
        timeAtStateStart = System.currentTimeMillis();
    }

    public void setGripPosition (double position) {
        if (invert) blueGrip.setPosition(position);
        else redGrip.setPosition(position);
    }

    public void setSwingPosition (double position) {
        if (invert) blueGripSwing.setPosition(position);
        else redGripSwing.setPosition(position);
    }

    public void tick(boolean grab, boolean release) {

        if (currentState == SideGripState.RETRACTED) {

            //Do this manually to make sure that both are always retracted in this state
            redGripSwing.setPosition(SWING_IN_POS);
            blueGripSwing.setPosition(SWING_IN_POS);

            //Only extend the grip servo on the side we're using
            setGripPosition(GRIP_OPEN_POS);

            if (grab) {
                currentState = SideGripState.READY;
                safeToMove = false;
            }
        }

        else if (currentState == SideGripState.READY) {
            setSwingPosition(SWING_DOWN_POS);
            if (System.currentTimeMillis() - timeAtStateStart > SWING_DOWN_MS) {
                currentState = SideGripState.GRIPPING;
            }
        }

        else if (currentState == SideGripState.GRIPPING) {
            setGripPosition(GRIP_CLOSED_POS);
            if (System.currentTimeMillis() - timeAtStateStart > GRIP_MS) {
                currentState = SideGripState.LIFTING;
            }

        }

        else if (currentState == SideGripState.LIFTING) {
            setSwingPosition(SWING_IN_POS);
            if (System.currentTimeMillis() - timeAtStateStart > SWING_UP_MS) {
                currentState = SideGripState.CARRYING;
                safeToMove = true;
            }
        }

        else if (currentState == SideGripState.CARRYING) {
            setSwingPosition(SWING_IN_POS);
            if (release) {
                currentState = SideGripState.RELEASING;
                safeToMove = false;
            }
        }

        else if (currentState == SideGripState.RELEASING) {
            setSwingPosition(SWING_RELEASE_POS);
            setGripPosition(GRIP_OPEN_POS);
            if (System.currentTimeMillis() - timeAtStateStart > RELEASING_MS) {
                currentState = SideGripState.RETRACTED;
                safeToMove = true;
            }
        }

        if (currentState != lastState) {
            // There was a state transition
            timeAtStateStart = System.currentTimeMillis();
        }
        lastState = currentState;
    }


}
