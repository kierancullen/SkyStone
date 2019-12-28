package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeController {

    double SWINGLEFT_STOW_POSITION;
    double SWINGRIGHT_STOW_POSITION;
    double SWINGLEFT_IDLE_POSITION;
    double SWINGRIGHT_IDLE_POSITION;
    double SWINGLEFT_INTAKE_POSITION;
    double SWINGRIGHT_INTAKE_POSITION;

    double IDLE_POWER;
    double INTAKE_POWER;

    long INTAKING_MS;

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

    OuttakeController outtake;

    public IntakeController(DcMotor intakeLeft,
                            DcMotor intakeRight,
                            Servo swingLeft,
                            Servo swingRight,
                            AnalogInput rangefinder,
                            OuttakeController outtake) {

        swingLeft.setPosition(SWINGLEFT_STOW_POSITION);
        swingRight.setPosition(SWINGRIGHT_STOW_POSITION);

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

    public void tick() {

        if (currentState == IntakeState.READY) {
            intakeLeft.setPower(IDLE_POWER);
            intakeRight.setPower(IDLE_POWER);
            swingLeft.setPosition(SWINGLEFT_IDLE_POSITION);
            swingRight.setPosition(SWINGRIGHT_IDLE_POSITION);
            if (rangefinderAlmost()) {
                currentState = IntakeState.ALMOST;
            }
        }

        else if (currentState == IntakeState.ALMOST) {
            if (rangefinderFull() && outtake.currentState == OuttakeController.OuttakeState.READY) {
                currentState = IntakeState.INTAKING;
            }
        }

        else if (currentState == IntakeState.INTAKING) {
            intakeLeft.setPower(INTAKE_POWER);
            intakeRight.setPower(INTAKE_POWER);
            swingLeft.setPosition(SWINGLEFT_INTAKE_POSITION);
            swingRight.setPosition(SWINGRIGHT_INTAKE_POSITION);
            if (System.currentTimeMillis() - timeAtStateStart > INTAKING_MS) {
                currentState = IntakeState.READY;
                outtake.currentState = OuttakeController.OuttakeState.GRABBING;
            }
        }


        if (currentState != lastState) {
            // There was a state transition
            timeAtStateStart = System.currentTimeMillis();
        }
        lastState = currentState;
    }


}
