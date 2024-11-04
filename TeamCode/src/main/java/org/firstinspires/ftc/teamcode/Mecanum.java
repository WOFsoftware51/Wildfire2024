/* Copyright (c) 2021 FIRST. All rights reserved.
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

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
//import org.opencv.core.RotatedRect;

import java.util.List;


@TeleOp(name="Mecanum")
public class Mecanum extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    ElapsedTime runtime = new ElapsedTime();
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    DcMotor motorArmRight;
    DcMotor motorArmLeft;
    Servo servoElbowLeft;
    Servo servoElbowRight;
    Servo servoWrist;
    Servo servoIntake;
    IMU imu;
    int portal1ViewId;
    int portal2ViewId;
    boolean overideLimit;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        motorFrontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        motorBackLeft  = hardwareMap.get(DcMotor.class, "backLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "frontRight");
        motorBackRight = hardwareMap.get(DcMotor.class, "backRight");
        motorArmRight  = hardwareMap.get(DcMotor.class, "armRight"); //Expansion
        motorArmLeft  = hardwareMap.get(DcMotor.class, "armLeft");
        servoElbowLeft = hardwareMap.servo.get("elbowRight");
        servoElbowRight = hardwareMap.servo.get("elbowLeft");
        servoWrist = hardwareMap.servo.get("servoWrist");
        servoIntake = hardwareMap.servo.get("servoIntake");
        imu = hardwareMap.get(IMU.class, "imu");



        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorArmRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorArmLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();



        double yaw = 0;
        String centric;
        double yp;
        double xp;

        boolean slow = false;
        int isFieldCentric = 1;
        imu.resetYaw();
        init_Orientation();
        overideLimit = false;

        servoElbowLeft.setPosition(0.15);
        servoElbowRight.setPosition(0.85);
        servoWrist.setPosition(0.65);


        motorArmLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArmRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();
        runtime.reset();
        if (isStopRequested()) return;


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            yaw = imu.getRobotYawPitchRollAngles().getYaw();



            if(gamepad2.a){
                servoElbowLeft.setPosition(0.22);
                servoElbowRight.setPosition(0.78);
                servoWrist.setPosition(0.78);
            }
            else if(gamepad2.b){ //Specimun Score
                servoElbowLeft.setPosition(0.57);
                servoElbowRight.setPosition(0.43);
                goToPosition(-130, motorArmLeft);
                goToPosition(-130, motorArmRight,motorArmLeft.getCurrentPosition());
                servoWrist.setPosition(0.55);
            }
            else if(gamepad2.x){
                servoElbowLeft.setPosition(0.85);
                servoElbowRight.setPosition(0.15);
            }
            else if(gamepad2.y){ //HP
                servoElbowLeft.setPosition(0.15);
                servoElbowRight.setPosition(0.85);
                servoWrist.setPosition(0.65);
                goToPosition(0, motorArmLeft);
                goToPosition(0, motorArmRight,motorArmLeft.getCurrentPosition());
            }
            else{
                if((motorArmLeft.getCurrentPosition()<-500 && gamepad2.left_stick_y<0) && !overideLimit){
                    motorArmLeft.setPower(0);
                    motorArmRight.setPower(0);
                }
                else if((motorArmLeft.getCurrentPosition()>0 && gamepad2.left_stick_y>0) && !overideLimit){
                    motorArmLeft.setPower(0);
                    motorArmRight.setPower(0);
                }
                else{
                    motorArmLeft.setPower(gamepad2.left_stick_y);
                    motorArmRight.setPower(gamepad2.left_stick_y);
                }
            }


            if(gamepad2.start){
                overideLimit = true;
            }
            else {
                overideLimit = false;
            }


            if(gamepad2.right_trigger>0.8){
                servoIntake.setPosition(1); //Close
            }
            else if(gamepad2.left_trigger>0.8){
                servoIntake.setPosition(0.5); //Open
            }



            if(gamepad2.back){
                motorArmRight.setMode(STOP_AND_RESET_ENCODER);
                motorArmLeft.setMode(STOP_AND_RESET_ENCODER);
            }
            else{
                motorArmRight.setMode(RUN_USING_ENCODER);
                motorArmLeft.setMode(RUN_USING_ENCODER);
            }





            if(gamepad2.right_bumper){
                servoWrist.setPosition(1);
            }
            else if(gamepad2.left_bumper){
                servoWrist.setPosition(0.1);
            }



            if(gamepad1.start && isFieldCentric == 0){
                isFieldCentric = 1;

            }
            if(gamepad1.back && isFieldCentric == 1){
                isFieldCentric = 0;

            }
            if(gamepad1.dpad_up)
            {
                imu.resetYaw();
            }

            if(isFieldCentric==1){
                centric = "Field";
            }
            else{
                centric = "Robot";
            }




            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double y = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double x = -gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x*0.8;

            if(gamepad1.right_bumper)
            {
                slow = true;
            }
            else
            {
                slow = false;
            }


            if(isFieldCentric==1){
                yp = (y*cos(yaw) - x*sin(yaw));
                xp = (x*cos(yaw) + y*sin(yaw));
            }
            else{
                yp = y;
                xp = x;
            }


            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (yp + xp + rx) / denominator;
            double backLeftPower = (yp - xp + rx) / denominator;
            double frontRightPower = (yp - xp - rx) / denominator;
            double backRightPower = (yp + xp - rx) / denominator;


            if(slow)
            {
                frontLeftPower = frontLeftPower*0.5;
                backLeftPower = backLeftPower*0.5;
                frontRightPower = frontRightPower*0.5;
                backRightPower = backRightPower*0.5;
            }




            // Send calculated power to wheels
            motorFrontLeft.setPower(frontLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackLeft.setPower(backLeftPower);
            motorBackRight.setPower(backRightPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.addData(centric, "Centric");
//            telemetry.addData("Elevator Encoder", motorElevator.getCurrentPosition());
            telemetry.addData("Left arm Encoders", motorArmLeft.getCurrentPosition());
            telemetry.addData("Right arm Encoders", motorArmRight.getCurrentPosition());
            telemetry.addData("Left Arm Mode",motorArmLeft.getMode());
            telemetry.addData("Right Arm Mode",motorArmRight.getMode());
            telemetry.addData("Right Arm target",motorArmRight.getTargetPosition());
            telemetry.addData("Yaw", yaw);


//            telemetry.addData("Left Arm Velocity", motorArmLeft.getPower());


            telemetry.update();
            sleep(20);

        }


    }

    public void init_Orientation(){
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public double cos(double degrees){
        return Math.cos(Math.toRadians(degrees));
    }
    public double sin(double degrees){
        return Math.sin(Math.toRadians(degrees));
    }


    public void resetEncoders(){
        motorArmRight.setMode(STOP_AND_RESET_ENCODER);
        motorArmLeft.setMode(STOP_AND_RESET_ENCODER);
    }



    public void goToPosition(int target, DcMotor motor) {
        int encoder = motor.getCurrentPosition();
        int difference = (target - encoder);

        if(difference > 10000)
        {
            motor.setPower(1.0);
        }
        else if(difference > 200)
        {
            motor.setPower(0.8);
        }
        else if(difference > 100)
        {
            motor.setPower(0.5);
        }
        else if(difference > 40)
        {
            motor.setPower(0.2);
        }
        else if(difference > 2.5)
        {
            motor.setPower(0.1);
        }
        else if(difference > -2.5)
        {
            motor.setPower(0.0);
        }
        else if(difference > -40)
        {
            motor.setPower(-0.2);
        }
        else if(difference > -100)
        {
            motor.setPower(-0.5);
        }
        else if(difference > -200)
        {
            motor.setPower(-0.8);
        }
        else if(difference > -10000)
        {
            motor.setPower(-1.0);
        }
    }
    public void goToPosition(int target, DcMotor motor, int encoder) {
        int difference = (target - encoder);

        if(difference > 10000)
        {
            motor.setPower(1.0);
        }
        else if(difference > 200)
        {
            motor.setPower(0.8);
        }
        else if(difference > 100)
        {
            motor.setPower(0.5);
        }
        else if(difference > 40)
        {
            motor.setPower(0.2);
        }
        else if(difference > 2.5)
        {
            motor.setPower(0.1);
        }
        else if(difference > -2.5)
        {
            motor.setPower(0.0);
        }
        else if(difference > -40)
        {
            motor.setPower(-0.2);
        }
        else if(difference > -100)
        {
            motor.setPower(-0.5);
        }
        else if(difference > -200)
        {
            motor.setPower(-0.8);
        }
        else if(difference > -10000)
        {
            motor.setPower(-1.0);
        }
    }
}
