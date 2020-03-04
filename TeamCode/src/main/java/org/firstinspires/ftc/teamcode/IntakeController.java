package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeController {


    double SWINGLEFT_INTAKE_POSITION = 0.72;
    double SWINGRIGHT_INTAKE_POSITION = 0.72;

    double SWINGLEFT_STOW_POSITION = 0.06;
    double SWINGRIGHT_STOW_POSITION = 0.06;

    double GATE_OPEN_POSITION = 0;
    double GATE_CLOSED_POSITION = 0.219;

    double INTAKE_POWER = 0.5;

    long INTAKING_MS = 1000;
    long RAMP_MS = 1000;
    long REVERSE_MS = 700;
    long FLOOR_MS = 700;
    long TRAP_MS = 500;

    DcMotor intakeLeft;
    DcMotor intakeRight;

    Servo swingLeft;
    Servo swingRight;

    Servo gate;

    DistanceSensor ramp;
    DistanceSensor floor;

    AnalogInput rangefinder;

    enum IntakeState {
        READY,
        RAMP,
        FLOOR,
        TRAPPED,
        REVERSING,
        STOWED
    }

    IntakeState currentState;
    IntakeState lastState;
    long timeAtStateStart;

    OuttakeController2 outtake;

    public IntakeController(DcMotor intakeLeft,
                            DcMotor intakeRight,
                            Servo swingLeft,
                            Servo swingRight,
                            Servo gate,
                            OuttakeController2 outtake,
                            DistanceSensor ramp,
                            DistanceSensor floor) {

        this.intakeLeft = intakeLeft;
        this.intakeRight = intakeRight;
        this.swingLeft = swingLeft;
        this.swingRight = swingRight;
        this.rangefinder = rangefinder;
        this.outtake = outtake;
        this.gate = gate;
        this.ramp = ramp;
        this.floor = floor;
    }

    public boolean onRamp() { return ramp.getDistance(DistanceUnit.MM) < 100;}
    public boolean onFloor() { return floor.getDistance(DistanceUnit.MM) < 100;}

    public boolean rangefinderFull() {
        return false;
    }

    public void start() {
        // Call once before calling calling tick. (In start() is the best place to do this)
        currentState = IntakeState.READY;
        lastState = IntakeState.READY;
        timeAtStateStart = System.currentTimeMillis();
    }

    public void tick(boolean go, boolean reverse, boolean stow) {

        if (currentState == IntakeState.READY) {
            outtake.readyForGrab = false;
            swingLeft.setPosition(SWINGLEFT_INTAKE_POSITION);
            swingRight.setPosition(SWINGLEFT_INTAKE_POSITION);

            gate.setPosition(GATE_OPEN_POSITION);

            if (outtake.currentState == OuttakeController2.OuttakeState.READY) {
                if (!reverse) {
                    intakeLeft.setPower(INTAKE_POWER);
                    intakeRight.setPower(INTAKE_POWER);
                }
                else {
                    intakeLeft.setPower(-INTAKE_POWER);
                    intakeRight.setPower(-INTAKE_POWER);
                }
            }
            else {
                intakeLeft.setPower(0);
                intakeRight.setPower(0);
            }

            if (onRamp() && outtake.currentState == OuttakeController2.OuttakeState.READY) {
                currentState = IntakeState.RAMP;
            }

            //In case we miss the ramp state somehow
            if (onFloor() && outtake.currentState == OuttakeController2.OuttakeState.READY) {
                currentState = IntakeState.FLOOR;
            }

            //If we manually trigger that the stone is in, just go right to the gate routine
            if (go) {
                currentState = IntakeState.FLOOR;
            }

            if (stow) {
                currentState = IntakeState.STOWED;
            }
        }

        else if (currentState == IntakeState.RAMP) {
            swingLeft.setPosition(SWINGLEFT_INTAKE_POSITION);
            swingRight.setPosition(SWINGLEFT_INTAKE_POSITION);
            if (!reverse) {
                intakeLeft.setPower(INTAKE_POWER);
                intakeRight.setPower(INTAKE_POWER);
            }
            else {
                intakeLeft.setPower(-INTAKE_POWER);
                intakeRight.setPower(-INTAKE_POWER);
            }

            if (onFloor()) {
                currentState = IntakeState.FLOOR;
            }
            //If we manually trigger that the stone is in, just go right to the gate routine
            if (go) {
                currentState = IntakeState.FLOOR;
            }

            //If we've been in this state for too long, the stone must be stuck
            if (System.currentTimeMillis() - timeAtStateStart > RAMP_MS) {
                currentState = IntakeState.REVERSING;
            }

        }

        else if (currentState == IntakeState.REVERSING) {
            intakeLeft.setPower(-INTAKE_POWER);
            intakeRight.setPower(-INTAKE_POWER);

            if (onFloor()) {
                currentState = IntakeState.FLOOR;
            }
            else if (!onRamp()) {
                currentState = IntakeState.READY;
            }

            if (go) {
                currentState = IntakeState.FLOOR;
            }

            if (System.currentTimeMillis() - timeAtStateStart > REVERSE_MS) {
                currentState = IntakeState.READY;
            }


        }

        else if (currentState == IntakeState.FLOOR) {

            if (System.currentTimeMillis() - timeAtStateStart > FLOOR_MS) {
                currentState = IntakeState.TRAPPED;
            }
        }

        else if (currentState == IntakeState.TRAPPED) {
            gate.setPosition(GATE_CLOSED_POSITION);
            intakeLeft.setPower(-INTAKE_POWER);
            intakeRight.setPower(-INTAKE_POWER);
            if (reverse) {
                gate.setPosition(GATE_OPEN_POSITION);
            }
            if (System.currentTimeMillis() - timeAtStateStart > TRAP_MS) {
                outtake.readyForGrab = true;

            }

            if (outtake.currentState == OuttakeController2.OuttakeState.GRABBING) {
                currentState = IntakeState.READY;
                outtake.readyForGrab = false;
            }
        }

        else if (currentState == IntakeState.STOWED) {
            gate.setPosition(GATE_CLOSED_POSITION);
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
            swingLeft.setPosition(SWINGLEFT_STOW_POSITION);
            swingRight.setPosition(SWINGRIGHT_STOW_POSITION);

            if (!stow) {
                currentState = IntakeState.READY;
            }


        }


        if (currentState != lastState) {
            // There was a state transition
            timeAtStateStart = System.currentTimeMillis();
        }
        lastState = currentState;
    }


}
