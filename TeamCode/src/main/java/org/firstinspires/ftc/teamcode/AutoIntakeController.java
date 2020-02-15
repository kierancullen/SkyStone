package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class AutoIntakeController {

    boolean readyForGrab = false;

    double SWINGLEFT_IDLE_POSITION = 0.72;//0.25
    double SWINGRIGHT_IDLE_POSITION = 0.72;//0.19
    double SWINGLEFT_INTAKE_POSITION = 0.72;
    double SWINGRIGHT_INTAKE_POSITION = 0.72;
    double SWINGLEFT_STOW_POSITION = 0.06;
    double SWINGRIGHT_STOW_POSITION = 0.06;

    double IDLE_POWER = 0.35;//0.3/1.5
    double INTAKE_POWER = 0.35;

    long INTAKING_MS = 5000;
    long REVERSE_START_MS = 2000; // after intaking starts
    long REVERSE_END_MS = 2500; // after intaking starts

    DcMotor intakeLeft;
    DcMotor intakeRight;
    Servo swingLeft;
    Servo swingRight;

    AnalogInput rangefinder;

    enum IntakeState {
        READY,
        ALMOST,
        STOP,
        INTAKING,
        RETRACTED
    }

    IntakeState currentState;
    IntakeState lastState;
    long timeAtStateStart;

    OuttakeController2 outtake;

    public AutoIntakeController(DcMotor intakeLeft,
                                DcMotor intakeRight,
                                Servo swingLeft,
                                Servo swingRight,
                                AnalogInput rangefinder,
                                OuttakeController2 outtake) {

        this.intakeLeft = intakeLeft;
        this.intakeRight = intakeRight;
        this.swingLeft = swingLeft;
        this.swingRight = swingRight;
        this.rangefinder = rangefinder;
        this.outtake = outtake;
    }

    public boolean rangefinderAlmost() {
        return false;
    }

    public boolean rangefinderFull() {
        return false;
    }

    public void start() {
        // Call once before calling calling tick. (In start() is the best place to do this)
        currentState = IntakeState.RETRACTED;
        lastState = IntakeState.RETRACTED;
        timeAtStateStart = System.currentTimeMillis();
    }

    public void tick(boolean go, boolean reverse, boolean retract) {
        if (retract) currentState = IntakeState.RETRACTED;

        if (currentState == IntakeState.RETRACTED) {
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
            swingLeft.setPosition(SWINGLEFT_STOW_POSITION);
            swingRight.setPosition(SWINGRIGHT_STOW_POSITION);
            if (!retract) {
                currentState = IntakeState.READY;
            }
        }

        if (currentState == IntakeState.READY) {
            swingLeft.setPosition(SWINGLEFT_IDLE_POSITION);
            swingRight.setPosition(SWINGRIGHT_IDLE_POSITION);
            if (outtake.currentState == OuttakeController2.OuttakeState.READY) {
                if (!reverse) {
                    intakeLeft.setPower(IDLE_POWER);
                    intakeRight.setPower(IDLE_POWER);
                }
                else {
                    intakeLeft.setPower(-IDLE_POWER);
                    intakeRight.setPower(-IDLE_POWER);
                }
            }
            else {
                intakeLeft.setPower(0);
                intakeRight.setPower(0);
            }
            //swingLeft.setPosition(SWINGLEFT_IDLE_POSITION);
            //swingRight.setPosition(SWINGRIGHT_IDLE_POSITION);
            if (rangefinderAlmost() || go) {
                currentState = IntakeState.INTAKING;
            }
        }

        else if (currentState == IntakeState.ALMOST) {
            if (rangefinderFull()) {
                currentState = IntakeState.INTAKING;
            }
        }

        else if (currentState == IntakeState.INTAKING) {
            long timeNow = System.currentTimeMillis();
            swingLeft.setPosition(SWINGLEFT_INTAKE_POSITION);
            swingRight.setPosition(SWINGRIGHT_INTAKE_POSITION);
            if ((timeNow - timeAtStateStart > REVERSE_START_MS && timeNow - timeAtStateStart < REVERSE_END_MS) || reverse) {
                intakeLeft.setPower(-INTAKE_POWER);
                intakeRight.setPower(-INTAKE_POWER);
            }
            else {
                intakeLeft.setPower(INTAKE_POWER);
                intakeRight.setPower(INTAKE_POWER);
            }
            //swingLeft.setPosition(SWINGLEFT_INTAKE_POSITION);
            //swingRight.setPosition(SWINGRIGHT_INTAKE_POSITION);
            if (System.currentTimeMillis() - timeAtStateStart > INTAKING_MS) {
                currentState = IntakeState.STOP;
                readyForGrab = true;
            }
        }

        else if (currentState == IntakeState.STOP) {
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
            if (System.currentTimeMillis() - timeAtStateStart > INTAKING_MS) currentState = IntakeState.READY;
        }


        if (currentState != lastState) {
            // There was a state transition
            timeAtStateStart = System.currentTimeMillis();
        }
        lastState = currentState;
    }


}
