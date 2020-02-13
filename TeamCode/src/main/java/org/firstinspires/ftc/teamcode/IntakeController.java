package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeController {

    boolean readyForGrab = false;

    double SWINGLEFT_IDLE_POSITION = 0.72;//0.25
    double SWINGRIGHT_IDLE_POSITION = 0.72;//0.19
    double SWINGLEFT_INTAKE_POSITION = 0.72;
    double SWINGRIGHT_INTAKE_POSITION = 0.72;

    double IDLE_POWER = 0.6;//0.3/1.5
    double INTAKE_POWER = 0.6;

    long INTAKING_MS = 1000;

    DcMotor intakeLeft;
    DcMotor intakeRight;
    Servo swingLeft;
    Servo swingRight;

    AnalogInput rangefinder;

    enum IntakeState {
        READY,
        ALMOST,
        INTAKING,
    }

    IntakeState currentState;
    IntakeState lastState;
    long timeAtStateStart;

    OuttakeController2 outtake;

    public IntakeController(DcMotor intakeLeft,
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
        currentState = IntakeState.READY;
        lastState = IntakeState.READY;
        timeAtStateStart = System.currentTimeMillis();
    }

    public void tick(boolean go, boolean reverse, float out) {

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
            swingLeft.setPosition(SWINGLEFT_INTAKE_POSITION);
            swingRight.setPosition(SWINGRIGHT_INTAKE_POSITION);
            if (!reverse) {
                intakeLeft.setPower(INTAKE_POWER);
                intakeRight.setPower(INTAKE_POWER);
            }
            else {
                intakeLeft.setPower(-INTAKE_POWER);
                intakeRight.setPower(-INTAKE_POWER);
            }

            if (System.currentTimeMillis() - timeAtStateStart > INTAKING_MS) {
                currentState = IntakeState.READY;
                readyForGrab = true;
            }
        }


        if (currentState != lastState) {
            // There was a state transition
            timeAtStateStart = System.currentTimeMillis();
        }
        lastState = currentState;
    }


}
