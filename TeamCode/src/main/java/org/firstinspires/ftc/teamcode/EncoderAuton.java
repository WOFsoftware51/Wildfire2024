package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;


@Autonomous(name = "EncoderAuton", group = "")
public class EncoderAuton extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    DcMotor motorArmRight;
    DcMotor motorArmLeft;
    DcMotor motorElevator;
    Servo servoWrist;
    Servo servoIntake;
    IMU imu;
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
        motorElevator = hardwareMap.get(DcMotor.class, "elevator");
        servoWrist = hardwareMap.servo.get("wrist");
        servoIntake = hardwareMap.servo.get("intake");
        imu = hardwareMap.get(IMU.class, "imu");


        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorElevator.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double yaw = 0;
        boolean slow = false;
        int isFieldCentric = 1;
        imu.resetYaw();
        init_Orientation();

        motorArmLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArmRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motorElevator.setMode(STOP_AND_RESET_ENCODER);
        motorArmLeft.setMode(STOP_AND_RESET_ENCODER);
        motorArmRight.setMode(STOP_AND_RESET_ENCODER);


        resetEncoders();

        waitForStart();

        if(opModeIsActive())
        {
            sleep(200);
            while(motorFrontLeft.getCurrentPosition() > -1000){ //about 24 inches/1 block
                drive(0, 0.3, 0);
            }
            drive(0, 0, 0);

            sleep(1000);
            while(motorFrontLeft.getCurrentPosition() > -2200){ //about 24 inches/1 block
                drive(0.3, 0, 0);
            }
            drive(0, 0, 0);

            sleep(1000);
            while(motorFrontLeft.getCurrentPosition() > -3500){ //about 24 inches/1 block
                drive(0, 0.3, 0);
            }
            drive(0, 0, 0);

            sleep(200);
            resetEncoders();

            sleep(1000);
            while(motorFrontLeft.getCurrentPosition() > -1000){
                drive(0, 0, 0.3);
            }

        }
        
    
}
    public void drive(double x, double y , double rx)
    {
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
    
    public void resetEncoders(){
        motorElevator.setMode(STOP_AND_RESET_ENCODER);
        motorArmLeft.setMode(STOP_AND_RESET_ENCODER);
        motorArmRight.setMode(STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(STOP_AND_RESET_ENCODER);
        sleep(1);
        motorElevator.setMode(RUN_USING_ENCODER);
        motorArmLeft.setMode(RUN_USING_ENCODER);
        motorArmRight.setMode(RUN_USING_ENCODER);
        motorFrontLeft.setMode(RUN_USING_ENCODER);
        motorFrontRight.setMode(RUN_USING_ENCODER);
        motorBackLeft.setMode(RUN_USING_ENCODER);
        motorBackRight.setMode(RUN_USING_ENCODER);
    }

    public void init_Orientation(){
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }
}


