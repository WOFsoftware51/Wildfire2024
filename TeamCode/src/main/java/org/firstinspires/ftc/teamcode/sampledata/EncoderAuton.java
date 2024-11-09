package org.firstinspires.ftc.teamcode.sampledata;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Disabled
@Autonomous(name = "EncoderAuton", group = "", preselectTeleOp = "Mecanum")
public class EncoderAuton extends LinearOpMode {

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
    boolean specimenScore;
    YawPitchRollAngles myRobotOrientation;

    @Override
    public void runOpMode() //throws InterruptedException 
    {
        // Declare our motors
        // Make sure your ID's match your configuration
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


        double yaw = 0;
        imu.resetYaw();
        specimenScore=false;


        motorArmLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArmRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sleep(3000);

        resetOperatorEncoders();
        resetDriveEncoders();

        servoElbowLeft.setPosition(0.15);
        servoElbowRight.setPosition(0.85);
        servoWrist.setPosition(0.1); //0.65
        servoIntake.setPosition(1);


        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();

        if(opModeIsActive()) { //24 inches = 1 block
//            sleep(200);
//            while(motorFrontLeft.getCurrentPosition() > -50){
//                drive(0, 0.3, 0);
//            }
//            drive(0,0,0);
//            while(!specimenScore){
//                specimenScore();
//            }
//            while(motorFrontLeft.getCurrentPosition() > -1400){
//                driveSpecimen(0, 0.3, 0);
//            }
//            driveSpecimen(0, 0, 0);
//            stopMotors();
//            while(motorFrontLeft.getCurrentPosition() < -1200){
//                servoIntake.setPosition(0);
//                drive(0, -0.3, 0);
//            }
//            drive(0,0,0);
//            while(motorFrontLeft.getCurrentPosition() < -1000){
//                driveArmDown(0, -0.3, 0);
//            }
//            driveArmDown(0, 0.0, 0);
//            stopMotors();
//
//            while(motorFrontLeft.getCurrentPosition() > -2200){
//                drive(0.3, 0, 0);
//            }
//            drive(0, 0, 0);
//
//            sleep(1000);
//            while(motorFrontLeft.getCurrentPosition() > -3500){
//                drive(0, 0.3, 0);
//            }
//            drive(0, 0, 0);
//
//            sleep(200);
//            resetDriveEncoders();
//            sleep(1000);
//            while(motorFrontLeft.getCurrentPosition() > -1000){
//                drive(0, 0, 0.3);
//            }

            sleep(200);
            while (motorFrontLeft.getCurrentPosition() > -1000) { //about 24 inches/1 block
                drive(0, 0.3, 0);
            }
            drive(0, 0, 0);

            sleep(200);
            while (motorFrontLeft.getCurrentPosition() > -2200) { //about 24 inches/1 block
                drive(0.3, 0, 0);
            }
            drive(0, 0, 0);

            sleep(200);
            while (motorFrontLeft.getCurrentPosition() > -3500) { //about 24 inches/1 block
                drive(0, 0.3, 0);
            }
            drive(0, 0, 0);
            resetDriveEncoders();

            sleep(200);
            while (motorFrontLeft.getCurrentPosition() < 950) {
                drive(0, 0, -0.3);
            }
            drive(0, 0, 0);
            resetDriveEncoders();

            sleep(200);
            while (motorFrontLeft.getCurrentPosition() > -550) {
                drive(0, 0.3, 0);
            }
            drive(0, 0, 0);
            resetDriveEncoders();

            sleep(200);
            while (motorFrontLeft.getCurrentPosition() > -4300) { //about 24 inches/1 block
                drive(0.3, 0, 0);
            }
            drive(0, 0, 0);
            sleep(200);
            while (motorFrontLeft.getCurrentPosition() > 0) { //about 24 inches/1 block
                drive(-0.3, 0, 0);
            }
            drive(0, 0, 0);
            resetDriveEncoders();

            sleep(200);
            while (motorFrontLeft.getCurrentPosition() > -400) {
                drive(0, 0.3, 0);
            }
            drive(0, 0, 0);
            resetDriveEncoders();

            sleep(200);
            while (motorFrontLeft.getCurrentPosition() > -4500) { //about 24 inches/1 block
                drive(0.3, 0, 0);
            }
            drive(0, 0, 0);
            sleep(200);
            while (motorFrontLeft.getCurrentPosition() > 0) { //about 24 inches/1 block
                drive(-0.3, 0, 0);
            }
            drive(0, 0, 0);
            resetDriveEncoders();

            sleep(200);
            while (motorFrontLeft.getCurrentPosition() > -400) {
                drive(0, 0.3, 0);
            }
            drive(0, 0, 0);

            sleep(200);
            while (motorFrontLeft.getCurrentPosition() > -3500) { //about 24 inches/1 block
                drive(0.3, 0, 0);
            }
            drive(0, 0, 0);
        }
}
    public void drive(double x, double y , double rx) {
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            motorFrontLeft.setPower(-frontLeftPower);
            motorFrontRight.setPower(-frontRightPower);
            motorBackLeft.setPower(-backLeftPower);
            motorBackRight.setPower(-backRightPower);
            telemetry.addData("Motor Front Left Encoder Pos", motorFrontLeft.getCurrentPosition());
            telemetry.update();
    }
    public void driveSpecimen(double x, double y , double rx) {
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;
        servoElbowLeft.setPosition(0.6);
        servoElbowRight.setPosition(0.4);
        servoWrist.setPosition(0.55);
        motorFrontLeft.setPower(-frontLeftPower);
        motorFrontRight.setPower(-frontRightPower);
        motorBackLeft.setPower(-backLeftPower);
        motorBackRight.setPower(-backRightPower);
        goToPosition(-130, motorArmLeft);
        goToPosition(-130, motorArmRight,motorArmLeft.getCurrentPosition());
        telemetry.addData("Motor Front Left Encoder Pos", motorFrontLeft.getCurrentPosition());
        telemetry.update();
    }
    public void driveArmDown(double x, double y , double rx) {
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;
        motorFrontLeft.setPower(-frontLeftPower);
        motorFrontRight.setPower(-frontRightPower);
        motorBackLeft.setPower(-backLeftPower);
        motorBackRight.setPower(-backRightPower);
        goToPosition(0, motorArmLeft);
        goToPosition(0, motorArmRight,motorArmLeft.getCurrentPosition());
        servoElbowLeft.setPosition(0.15);
        servoElbowRight.setPosition(0.85);
        servoWrist.setPosition(0.65);
        telemetry.addData("Motor Front Left Encoder Pos", motorFrontLeft.getCurrentPosition());
        telemetry.update();
    }

    public void stopMotors(){
        motorArmLeft.setPower(0);
        motorArmRight.setPower(0);
    }

    public void specimenScore() {
        servoElbowLeft.setPosition(0.6);
        servoElbowRight.setPosition(0.4);
        goToPosition(-130, motorArmLeft);
        goToPosition(-130, motorArmRight,motorArmLeft.getCurrentPosition());
        servoWrist.setPosition(0.55);
        if(((motorArmLeft.getCurrentPosition()-5)<-130)){
            specimenScore=true;
        }
    }
    public void armDown() {
        servoElbowLeft.setPosition(0.15);
        servoElbowRight.setPosition(0.85);
        servoWrist.setPosition(0.65);
        goToPosition(0, motorArmLeft);
        goToPosition(0, motorArmRight,motorArmLeft.getCurrentPosition());
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
            motor.setPower(0.4);
        }
        else if(difference > 15)
        {
            motor.setPower(0.3);
        }
        else if(difference > 2.5)
        {
            motor.setPower(0);
        }
        else if(difference > -2.5)
        {
            motor.setPower(0);
        }
        else if(difference > -15)
        {
            motor.setPower(-0.3);
        }
        else if(difference > -40)
        {
            motor.setPower(-0.4);
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
            motor.setPower(0.4);
        }
        else if(difference > 15)
        {
            motor.setPower(0.3);
        }
        else if(difference > 2.5)
        {
            motor.setPower(0);
        }
        else if(difference > -2.5)
        {
            motor.setPower(0);
        }
        else if(difference > -15)
        {
            motor.setPower(-0.3);
        }
        else if(difference > -40)
        {
            motor.setPower(-0.4);
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

    public void resetOperatorEncoders(){
        motorArmLeft.setMode(STOP_AND_RESET_ENCODER);
        motorArmRight.setMode(STOP_AND_RESET_ENCODER);
        sleep(1);
        motorArmLeft.setMode(RUN_USING_ENCODER);
        motorArmRight.setMode(RUN_USING_ENCODER);
    }
    public void resetDriveEncoders(){
        motorFrontLeft.setMode(STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(STOP_AND_RESET_ENCODER);
        sleep(1);
        motorFrontLeft.setMode(RUN_USING_ENCODER);
        motorFrontRight.setMode(RUN_USING_ENCODER);
        motorBackLeft.setMode(RUN_USING_ENCODER);
        motorBackRight.setMode(RUN_USING_ENCODER);
    }


}


