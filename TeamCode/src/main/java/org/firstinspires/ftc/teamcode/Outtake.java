package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Outtake {
    private DcMotor shooterLeft, shooterRight;
    private Servo flickLeft, flickRight;
    private ElapsedTime stateTimer = new ElapsedTime();

    private enum ShooterState {
        IDLE, SPIN_UP, READY, FIRING, RESET
    }
    private ShooterState state = ShooterState.IDLE;

    // Constants
    private final double TARGET_POWER = 0.6;
    private final double SPIN_UP_TIME = 1.0; // seconds
    private final double FLICK_TIME = 0.3;
    private final double RESET_TIME = 0.3;

    // Servo positions
    private final double FLICK_LEFT_REST = 0.85;
    private final double FLICK_LEFT_FIRE = 0.55;
    private final double FLICK_RIGHT_REST = 0.05;
    private final double FLICK_RIGHT_FIRE = 0.35;

    public void init(HardwareMap hardwareMap) {
        shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotor.class, "shooterRight");
        flickLeft = hardwareMap.get(Servo.class, "FlickLeft");
        flickRight = hardwareMap.get(Servo.class, "FlickRight");

        shooterRight.setDirection(DcMotor.Direction.REVERSE);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flickLeft.setPosition(FLICK_LEFT_REST);
        flickRight.setPosition(FLICK_RIGHT_REST);

        state = ShooterState.IDLE;
    }

    public void update() {
        switch (state) {
            case IDLE:

            case READY:
                break;

            case SPIN_UP:
                shooterLeft.setPower(TARGET_POWER);
                shooterRight.setPower(TARGET_POWER);

                if (stateTimer.seconds() >= SPIN_UP_TIME) {
                    state = ShooterState.READY;
                    stateTimer.reset();
                }
                break;

            case FIRING:
                flickLeft.setPosition(FLICK_LEFT_FIRE);
                flickRight.setPosition(FLICK_RIGHT_FIRE);

                if (stateTimer.seconds() >= FLICK_TIME) {
                    state = ShooterState.RESET;
                    stateTimer.reset();
                }
                break;

            case RESET:
                flickLeft.setPosition(FLICK_LEFT_REST);
                flickRight.setPosition(FLICK_RIGHT_REST);

                if (stateTimer.seconds() >= RESET_TIME) {
                    state = ShooterState.IDLE;
                    stopShooter();
                }
                break;
        }
    }

    public void startShooter() {
        if (state == ShooterState.IDLE) {
            state = ShooterState.SPIN_UP;
            stateTimer.reset();
        }
    }

    public void fire() {
        if (state == ShooterState.READY) {
            state = ShooterState.FIRING;
            stateTimer.reset();
        }
    }

    public void stopShooter() {
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
    }

    public boolean isBusy() {
        return state != ShooterState.IDLE;
    }

    public boolean isReady() {
        return state == ShooterState.READY;
    }
}
