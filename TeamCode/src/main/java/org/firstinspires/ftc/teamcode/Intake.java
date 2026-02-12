package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    private DcMotor intake;
    private ElapsedTime stateTimer = new ElapsedTime();

    private enum IntakeState {
        IDLE, INTAKING, OUTTAKING
    }
    private IntakeState state = IntakeState.IDLE;

    private double intakeTime = 0;

    public void init(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        state = IntakeState.IDLE;
    }

    public void update() {
        switch (state) {
            case IDLE:
                intake.setPower(0);
                break;

            case INTAKING:
                intake.setPower(-1.0);

                if (intakeTime > 0 && stateTimer.seconds() >= intakeTime) {
                    state = IntakeState.IDLE;
                    intakeTime = 0;
                }
                break;

            case OUTTAKING:
                intake.setPower(1.0);

                if (intakeTime > 0 && stateTimer.seconds() >= intakeTime) {
                    state = IntakeState.IDLE;
                    intakeTime = 0;
                }
                break;
        }
    }

    public void startIntake(double duration) {
        state = IntakeState.INTAKING;
        intakeTime = duration;
        stateTimer.reset();
    }

    public void startOuttake(double duration) {
        state = IntakeState.OUTTAKING;
        intakeTime = duration;
        stateTimer.reset();
    }

    public void stop() {
        state = IntakeState.IDLE;
        intakeTime = 0;
    }

    public boolean isBusy() {
        return state != IntakeState.IDLE;
    }
}
