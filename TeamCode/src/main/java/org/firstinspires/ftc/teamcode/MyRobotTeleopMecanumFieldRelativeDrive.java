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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "MAIN Drive (Dual Mode)", group = "Robot")
public class MyRobotTeleopMecanumFieldRelativeDrive extends OpMode {

    // Drive motors
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    // Mechanisms
    DcMotor shooterLeft;
    DcMotor intake;
    DcMotor shooterRight;

    // Flick servos
    Servo flickLeft;
    Servo flickRight;

    // IMU (for heading)
    IMU imu;

    @Override
    public void init() {
        // Map hardware
        frontLeftDrive = hardwareMap.get(DcMotor.class, "driveLeftFront");
        frontRightDrive = hardwareMap.get(DcMotor.class, "driveRightFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "driveLeftRear");
        backRightDrive = hardwareMap.get(DcMotor.class, "driveRightRear");

        shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooterRight = hardwareMap.get(DcMotor.class, "shooterRight");

        flickLeft = hardwareMap.get(Servo.class, "FlickLeft");
        flickRight = hardwareMap.get(Servo.class, "FlickRight");

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // Reverse one shooter motor so both shooters spin the same
        shooterRight.setDirection(DcMotor.Direction.REVERSE);

        // Starting positions
        flickLeft.setPosition(0.05);
        flickRight.setPosition(0.85);

        // Encoders for more consistent drive response
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Brake
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @Override
    public void loop() {
        telemetry.addLine("B: reset yaw");
        telemetry.addLine("Hold LT: robot-centric");
        telemetry.addLine("Hold RT: slow mode");

        // Intake controls (as coded):
        // - Y on either controller: intake in
        // - X on gamepad1 only: intake out
        // - D-pad up on gamepad1 only: stop intake
        if ((gamepad1.y) || (gamepad2.y)) {
            intake.setPower(-0.7);
        } else if (gamepad1.dpad_up) {
            intake.setPower(0);
        } else if (gamepad1.x) {
            intake.setPower(0.7);
        }

        // Shooter controls (gamepad1):
        // - A: shooters on
        // - D-pad down: shooters off
        if (gamepad1.a) {
            shooterRight.setPower(1);
            shooterLeft.setPower(1);
        }
        if (gamepad1.dpad_down) {
            shooterLeft.setPower(0);
            shooterRight.setPower(0);
        }

        // Flickers (hold to actuate, release to return):
        if ((gamepad2.left_bumper) || (gamepad1.left_bumper)) {
            flickRight.setPosition(0.55);
        } else {
            flickRight.setPosition(0.85);
        }

        if ((gamepad2.right_bumper) || (gamepad1.right_bumper)) {
            flickLeft.setPosition(0.35);
        } else {
            flickLeft.setPosition(0.05);
        }

        // Reset heading (yaw) to zero from the current robot direction
        if (gamepad1.b) {
            imu.resetYaw();
        }

        // Drive mode selection
        if (gamepad1.left_trigger > .5) {
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            telemetry.addLine("Drive: Robot Centric");
        } else {
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            telemetry.addLine("Drive: Field Centric");
        }

        telemetry.addLine("");
        telemetry.addLine("Intake: Y=in, X=out, DPadUp=stop");
        telemetry.addLine("Shooter: A=on, DPadDown=off");
        telemetry.addLine("Flick: hold bumpers");
    }

    // Field-centric wrapper: rotate the driver input by the robot heading
    private void driveFieldRelative(double forward, double right, double rotate) {
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        drive(newForward, newRight, rotate);
    }

    // Mecanum mix + normalization
    public void drive(double forward, double right, double rotate) {
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = -forward + right - rotate;
        double backLeftPower = -forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed;

        // Slow mode while holding RT
        if (gamepad1.right_trigger > 0.5) {
            maxSpeed = 0.5;
            telemetry.addLine("Speed: Slow");
        } else {
            maxSpeed = 1.0;
            telemetry.addLine("Speed: Normal");
        }

        // Normalize so no wheel command exceeds 1.0 (keeps proportions the same)
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }
}
