package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeControl {
    private DcMotor intake;


    private enum IntakeState{
        IDLE,
        INTAKING,
        REVERSE,
        OFF

    }
    private IntakeState intakeState;

    // --------------- GATE LOGIC ----------------
    private double intake_forward_power = -1;
    private double intake_reverse_power = 1;



    private int intakePower = 0;
    private boolean changePower = false;

    public void init(HardwareMap hwMap){
        intake=hwMap.get(DcMotor.class,"intake");
        intakeState = IntakeState.IDLE;
        intake.setPower(0);
    }

    public void update(){
        switch (intakeState){
            case IDLE:
                if (changePower) {
                    if (intakePower < 0) {
                        intakeState = IntakeState.INTAKING;
                    } else if (intakePower > 0) {
                        intakeState = IntakeState.REVERSE;
                    } else {
                        intakeState = IntakeState.OFF;
                    }
                }
                break;
            case INTAKING:
                intake.setPower(intake_forward_power);
                intakeState=IntakeState.IDLE;
                changePower=false;
                break;
            case REVERSE:
                intake.setPower(intake_reverse_power);
                intakeState=IntakeState.IDLE;
                changePower=false;
                break;
            case OFF:
                intake.setPower(0);
                intakeState=IntakeState.IDLE;
                changePower=false;
                break;
        }
    }
    public void setIntakePower(int requestedPower){
        if (intakeState== IntakeState.IDLE){
            intakePower=requestedPower;
            changePower=true;
        }
    }

    public boolean isBusy(){
        return intakeState != IntakeState.IDLE;
    }
}

