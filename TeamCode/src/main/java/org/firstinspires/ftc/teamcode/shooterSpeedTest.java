package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class shooterSpeedTest extends LinearOpMode {
    DcMotorEx shooterLeft;
    DcMotor intake;
    DcMotorEx shooterRight;
    Servo FlickLeft;
    Servo FlickRight;



    @Override public void runOpMode(){
        double flyWheelSpeed=.6;
        double leftPos=0;
        double rightPos=0;

        FlickLeft = hardwareMap.get(Servo.class, "FlickLeft" );
        FlickRight = hardwareMap.get(Servo.class, "FlickRight");

        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        shooterRight.setDirection(DcMotor.Direction.REVERSE);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()){

            if ((gamepad1.a)||(gamepad2.a)){
                shooterRight.setPower(flyWheelSpeed);
                shooterLeft.setPower(flyWheelSpeed);

            }
            telemetry.addLine("Right Speed: " + String.valueOf(shooterRight.getVelocity()));
            telemetry.addLine("Left Speed: " + String.valueOf(shooterLeft.getVelocity()));


            if ((gamepad1.dpad_down)||(gamepad2.dpad_down)){
                shooterLeft.setPower(0);
                shooterRight.setPower(0);
            }


            if ((gamepad2.right_bumper)||(gamepad1.right_bumper)){
                //move RIGHT servo to the rotated position like 90 degrees prob
                FlickRight.setPosition(0.3);

            }else{
                //move RIGHT servo back to original position
                FlickRight.setPosition(0.05);
            }
            if ((gamepad2.left_bumper) || (gamepad1.left_bumper)){
                //move LEFT servo to the rotated position like 90 degrees prob
                FlickLeft.setPosition(0.575);
            }else{
                //move LEFT servo back to original position
                FlickLeft.setPosition(0.87);
            }

            if ((gamepad1.y)||(gamepad2.y)) {
                intake.setPower(-1);

            } else if ((gamepad1.dpad_up)||(gamepad2.dpad_up)){
                intake.setPower(0);

            } else if ((gamepad1.x)||(gamepad2.x)){
                intake.setPower(1);
            }


        }
    }
}
