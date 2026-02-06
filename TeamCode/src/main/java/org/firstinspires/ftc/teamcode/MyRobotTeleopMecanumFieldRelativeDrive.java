/* Copyright (c) 2025 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
/*
 * This OpMode illustrates how to program your robot to drive field relative.  This means
 * that the robot drives the direction you push the joystick regardless of the current orientation
 * of the robot.
 *
 * This OpMode assumes that you have four mecanum wheels each on its own motor named:
 *   front_left_motor, front_right_motor, back_left_motor, back_right_motor
 *
 *   and that the left motors are flipped such that when they turn clockwise the wheel moves backwards
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 */
@TeleOp(name = "MAIN Drive (Dual Mode)", group = "Robot")

public class MyRobotTeleopMecanumFieldRelativeDrive extends OpMode {
    // This declares the four motors needed
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor shooterLeft;
    DcMotor intake;
    DcMotor shooterRight;
    Servo FlickLeft;
    Servo FlickRight;

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;

    @Override
    public void init() {

        //front left and front right are switched for tesitng currently
        frontRightDrive = hardwareMap.get(DcMotor.class, "driveLeftFront");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "driveRightFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "driveLeftRear");
        backRightDrive = hardwareMap.get(DcMotor.class, "driveRightRear");


        shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooterRight = hardwareMap.get(DcMotor.class, "shooterRight");

        FlickLeft = hardwareMap.get(Servo.class, "FlickLeft" );
        FlickRight = hardwareMap.get(Servo.class, "FlickRight");

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);



        //reverses one of the shooter motors so we run both same direction to shoot
        shooterRight.setDirection(DcMotor.Direction.REVERSE);

        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        FlickLeft.setPosition(0.85);
        FlickRight.setPosition(0.05);



        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @Override
    public void loop() {
        telemetry.addLine("Press B to reset Yaw");
        telemetry.addLine("Hold left trigger to drive in robot relative");
        telemetry.addLine("Hold right trigger to drive in 50% speed");




        //intake toggles
        if ((gamepad1.y)||(gamepad2.y)) {
            intake.setPower(-0.7);

        } else if ((gamepad1.dpad_up)||(gamepad2.dpad_up)){
            intake.setPower(0);

        } else if ((gamepad1.x)||(gamepad2.x)){
            intake.setPower(0.7);
        }






        //shooter toggle
        if ((gamepad1.a)||(gamepad2.a)){
            shooterRight.setPower(0.85);
            shooterLeft.setPower(0.85);
        }
        if ((gamepad1.dpad_down)||(gamepad2.dpad_down)){
            shooterLeft.setPower(0);
            shooterRight.setPower(0);
        }





        if ((gamepad2.right_bumper)||(gamepad1.right_bumper)){
            //move RIGHT servo to the rotated position like 90 degrees prob


            FlickRight.setPosition(0.35);
            assert true;
        }else{
            //move RIGHT servo back to original position
            FlickRight.setPosition(0.05);
            assert true;
        }

        if ((gamepad2.left_bumper) || (gamepad1.left_bumper)){
            //move LEFT servo to the rotated position like 90 degrees prob


            FlickLeft.setPosition(0.55);

        }else{
            //move LEFT servo back to original position
            FlickLeft.setPosition(0.85);

        }





        // If you press the B button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        if (gamepad1.b) {
            imu.resetYaw();

        }
        // If you press the left trigger, you get a drive from the point of view of the robot
        // (much like driving an RC vehicle)
        if (gamepad1.left_trigger>.5) {
            drive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            telemetry.addLine("Robot Centric");
        } else {
            driveFieldRelative(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            telemetry.addLine("Field Centric");
        }
        //adds telemetry for function op
        telemetry.addLine("");
        telemetry.addLine("Y or DPAD UP will toggle the intake");
        telemetry.addLine("Use Bumpers to toggle left and right shooters");
    }




    // This routine drives the robot field relative
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = -(AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = -forward + right - rotate;
        double backLeftPower = -forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed=0.5;

        if (gamepad1.right_trigger>0.5) {
            maxSpeed=0.5;
            telemetry.addLine("Speed: Slow");
        }else{
            maxSpeed = 1.0;  // make this slower if u wanna go slower
            telemetry.addLine("Speed: Normal");
        }

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }
}
