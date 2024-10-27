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
//    VisionPortal aprilTagCamera;
//    VisionPortal colorSensorCamera;
    AprilTagProcessor aprilTagProcessor;
    ColorBlobLocatorProcessor colorLocator;
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
    YawPitchRollAngles myRobotOrientation;
    int portal1ViewId;
    int portal2ViewId;

//    private Position cameraPosition = new Position(DistanceUnit.INCH,
//            0, 0, 0, 0);
//    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
//            0, -90, 0, 0);



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
        initAprilTag();


        colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();


        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        int[] viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.VERTICAL);
        portal1ViewId = viewIds[0];
        portal2ViewId = viewIds[1];

//        aprilTagCamera = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .setLiveViewContainerId(portal1ViewId)
//                .addProcessor(aprilTagProcessor)
//                .build();


//        colorSensorCamera = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
//                .setLiveViewContainerId(portal2ViewId)
//                .addProcessor(colorLocator)
//                .setCameraResolution(new Size(320, 240))
//                .build();

        double yaw = 0;
//        double pitch = 0;
//        double roll = 0;
        String centric;
        double yp;
        double xp;

        boolean slow = false;
        int isFieldCentric = 1;
        imu.resetYaw();
        init_Orientation();


//        int elevatorEncoderMAX = 5333;
//        int armEncoderFLAT = 2333;


//        motorElevator.setMode(STOP_AND_RESET_ENCODER);
//        motorArmLeft.setMode(STOP_AND_RESET_ENCODER);
//        motorArmRight.setMode(STOP_AND_RESET_ENCODER);

        motorArmLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArmRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();
        runtime.reset();
        if (isStopRequested()) return;


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetryAprilTag();
            myRobotOrientation = imu.getRobotYawPitchRollAngles();
            yaw = imu.getRobotYawPitchRollAngles().getYaw();
//            pitch = myRobotOrientation.getPitch(AngleUnit.DEGREES);
//            roll  = myRobotOrientation.getRoll(AngleUnit.DEGREES);

//            motorElevator.setMode(RUN_USING_ENCODER);
//            motorArmLeft.setMode(RUN_USING_ENCODER);
//            motorArmRight.setMode(RUN_USING_ENCODER);


//            motorElevator.setPower(gamepad2.right_stick_y);


            if(gamepad2.a){ //high
                servoElbowLeft.setPosition(0.4);
                servoElbowRight.setPosition(0.6);
            }
            else if(gamepad2.b){ //low
                servoElbowLeft.setPosition(0.5);
                servoElbowRight.setPosition(0.5);
            }
            else if(gamepad2.x){
                servoElbowLeft.setPosition(0.85);
                servoElbowRight.setPosition(0.15);
            }
            else if(gamepad2.y){
                servoElbowLeft.setPosition(0.15);
                servoElbowRight.setPosition(0.85);
            }

            if(gamepad2.right_trigger>0.8){
                servoIntake.setPosition(0);
            }
            else if(gamepad2.left_trigger>0.8){
                servoIntake.setPosition(1);
            }

            if(gamepad2.right_bumper){
//                servoWrist.setPosition(0.5);
            }

            if(gamepad2.y){
                motorArmRight.setMode(RUN_TO_POSITION);
                motorArmLeft.setMode(RUN_TO_POSITION);
                motorArmRight.setTargetPosition(50);
                motorArmLeft.setTargetPosition(-50);
            }
            else{
                motorArmLeft.setPower(-gamepad2.left_stick_y);
                motorArmRight.setPower(gamepad2.left_stick_y);
                motorArmRight.setMode(RUN_USING_ENCODER);
                motorArmLeft.setMode(RUN_USING_ENCODER);
            }

            if(gamepad2.right_trigger>0.8){
                servoIntake.setPosition(1);
            }
            else if(gamepad2.left_trigger>0.8){
                servoIntake.setPosition(0);
            }

            if(gamepad2.right_bumper){
                servoWrist.setPosition(1);
            }
            else if(gamepad2.left_bumper){
                servoWrist.setPosition(0);
            }
            else if(gamepad2.x){
                servoWrist.setPosition(0.5);
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
            double y = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x*0.8;

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

//            // Normalize the values so no wheel power exceeds 100%
//            // This ensures that the robot maintains the desired motion.
//            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
//            max = Math.max(max, Math.abs(backLeftPower));
//            max = Math.max(max, Math.abs(backRightPower));
//
////            if (max > 1.0) {
//                frontLeftPower  /= max;
//                frontRightPower /= max;
//                backLeftPower   /= max;
//                backRightPower /= max;
//            }

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



            // APRILTAGS Main Loop
            // WARNING:  To be able to view the stream preview on the Driver Station, this code runs in INIT mode.

//                telemetry.addData("preview on/off", "... Camera Stream\n");

                // Read the current list
                List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

                ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);  // filter out very small blobs.


//                telemetry.addLine(" Area Density Aspect  Center");

                // Display the size (area) and center location for each Blob.
//                for(ColorBlobLocatorProcessor.Blob b : blobs)
//                {
//                    RotatedRect boxFit = b.getBoxFit();
////                    telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
////                            b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y));
//                }

                // Just show some basic telemetry to demonstrate both processors are working in parallel
                // on their respective cameras. If you want to see more detail about the information you
                // can get back from the processor, you should look at ConceptAprilTag.
//                telemetry.addData("Number of tags in Camera 2", aprilTagProcessor.getDetections().size());
    //          telemetry.addData("Number of tags in Camera 1", aprilTagProcessor2.getDetections().size());


            telemetry.update();
            sleep(20);

        }


    }

    public void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.metadata != null) {
//                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
//                        detection.robotPose.getPosition().x,
//                        detection.robotPose.getPosition().y,
//                        detection.robotPose.getPosition().z));
//                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
//                        detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
//                        detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
//                        detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
//            }
//            else{
//                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
//            }
//        }   // end for() loop

        // Add "key" information to telemetry
//        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
//        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

    }


    public void init_Orientation(){
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public double cos(double degrees){
        return Math.cos(Math.toRadians(degrees));
    }
    public double sin(double degrees){
        return Math.sin(Math.toRadians(degrees));
    }



    public void initAprilTag() {

        // Create the AprilTag processor.
        aprilTagProcessor = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
//                .setCameraPose(cameraPosition, cameraOrientation)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();


    }   // end method initAprilTag()

    public void resetEncoders(){
//        motorElevator.setTargetPosition(0);
    }


}
