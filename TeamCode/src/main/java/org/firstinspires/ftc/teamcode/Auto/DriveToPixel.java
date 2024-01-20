package org.firstinspires.ftc.teamcode.Auto;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;
@TeleOp(name = "DriveToPixel")

public class DriveToPixel extends LinearOpMode {
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;// Used to hold the data for a detected AprilTag
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    // height of wanted camera resolution
    private double degrees =0;
    private boolean isStraight = false;
    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.25;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 1066.15385;
    private static final double FOV = 40;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "iSwearToGod.tflite";

    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    // Define the labels recognized in the model for TFOD (must be in training order!)

    private static final String[] LABELS = {
            "whitePixel",
    };


    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal1;
    private VisionPortal visionPortal2;
    final double DESIRED_DISTANCE = 8.0; // how close
    final double SPEED_GAIN  =  0.03  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double TURN_GAIN   =  0.02  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_SPEED = 0.7;   //  Clip the approach speed to this max value

    final double MAX_AUTO_TURN  = 0.8;   //  Clip the turn speed to this max value

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;


    @Override
    public void runOpMode() {
        double drive=0;
        double turn=0;
        double strafe=0;


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBottomDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBottomDrive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
//        initAprilTag();

        // Wait for the DS start button to be touched.
        if (USE_WEBCAM)
            setManualExposure1(6, 250);  // Use low exposure time to reduce motion blur

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        motorTest auto = new motorTest(leftFrontDrive,rightFrontDrive,leftBackDrive,rightBackDrive);
        waitForStart();

//        auto.moveForward(0.6,3000);
//        auto.turnleft(0.6,1450);
//        aprilTagAuto(drive,strafe,turn);
//        visionPortal1.close();
//        initTfod();
//        auto.turnRight(0.6, 2900);
//        auto.moveForward(0.6,500);
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                trackPixelAuto(drive, turn, strafe);

                // Share the CPU.
                sleep(20);
            }
        }



        // Save more CPU resources when camera is no longer needed.
        visionPortal2.close();

    }   // end runOpMode()
    public void aprilTagAuto(double drive, double strafe, double turn) {
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < 7) {
            boolean targetFound = false;
            boolean lostIt = false;
            desiredTag = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
                telemetry.addData("time",  timer.seconds());
            } else {
                telemetry.addData("\n>", "Drive using joysticks to find valid target\n");
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;
//                double yawError = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive = -1 * Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                strafe = Range.clip(-headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
//                turn = Range.clip(yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                setDegrees(desiredTag.ftcPose.bearing);
                if (rangeError < 0.5 || rangeError > -0.5) {
                    setisStraight(true);
                } else {
                    setisStraight(false);
                }
                lostIt = true;

                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
//            else if (lostIt && !targetFound) {
//                if (isStraight) {
//                    setDegrees(0);
//                    continue;
//                }
//                double rangeError = 0;
//                double headingError = getDegrees();
//                double yawError = 0;
//                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
//                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
//                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
//                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
//            }
//            else {
//
//                // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
//                drive = -gamepad1.left_stick_y / 1.5;  // Reduce drive rate to 50%.
//                strafe = gamepad1.right_stick_x / 1.5;  // Reduce strafe rate to 50%.
//                turn = -gamepad1.left_stick_x / 2.0;  // Reduce turn rate to 33%.
//                telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
//            }
            telemetry.update();

            // Apply desired  motions to the drivetrain.

            moveRobot(drive, strafe, turn);
            sleep(10);
        }
    }
 // Set true to use a webcam, or false for a phone camera
 public void moveRobot(double x, double y, double yaw) {

     // Calculate wheel powers.
     double leftFrontPower    =  x -y -yaw;
     double rightFrontPower   =  x +y +yaw;
     double leftBackPower     =  x +y -yaw;
     double rightBackPower    =  x -y +yaw;

     // Normalize wheel powers to be less than 1.0
     double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
     max = Math.max(max, Math.abs(leftBackPower));
     max = Math.max(max, Math.abs(rightBackPower));

     if (max > 1.0) {
         leftFrontPower /= max;
         rightFrontPower /= max;
         leftBackPower /= max;
         rightBackPower /= max;
     }

     // Send powers to the wheels.
     leftFrontDrive.setPower(leftFrontPower);
     rightFrontDrive.setPower(rightFrontPower);
     leftBackDrive.setPower(leftBackPower);
     rightBackDrive.setPower(rightBackPower);
 }

    public void trackPixelAuto(double drive, double turn, double strafe){
        boolean targetFound = false;
        Recognition rec = null;

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {



            targetFound = true;
            rec = recognition;

        }
        if(targetFound){
            double x = (rec.getLeft() + rec.getRight()) / 2;
            double y = (rec.getTop() + rec.getBottom()) / 2;
            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", rec.getLabel(), rec.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", rec.getWidth(), rec.getHeight());
            telemetry.addData("Distance to Pixle: ", getDistance(rec.getWidth()));
            telemetry.addData("Angle to object: ", getAngleTarget(middleOfObject(rec.getWidth(), rec.getLeft())));

        }else {
            telemetry.addData("\n>","Drive using joysticks to find valid target\n");
        }
        if (targetFound) {
            double  rangeError      = 6 - getDistance(rec.getWidth());
            double  headingError    = -1 * getAngleTarget(middleOfObject(rec.getWidth(),rec.getLeft()));
            strafe = 0;
            drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
        }
        else{

            drive  = gamepad1.left_stick_y  / 1.5;  // Reduce drive rate to 50%.
            strafe = -gamepad1.left_stick_x  / 1.5;  // Reduce strafe rate to 50%.
            turn   = gamepad1.right_stick_x / 2.0;  // Reduce turn rate to 33%.
            telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

        }
        telemetry.update();
        moveRobot(drive, turn, strafe);

        // Push telemetry to the Driver Station.

        // Save CPU resources; can resume streaming when needed.
        if (gamepad1.dpad_down) {
            visionPortal2.stopStreaming();
        } else if (gamepad1.dpad_up) {
            visionPortal2.resumeStreaming();
        }
    }
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal1 = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal1 = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.

        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.

                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(modelFilePath)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal2 = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        visionPortal2.setProcessorEnabled(tfod, true);
    }


    private void    setManualExposure1(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal1 == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
      if (visionPortal1.getCameraState() != VisionPortal.CameraState.STREAMING) {
        telemetry.addData("Camera", "Waiting");
        telemetry.update();
        while (!isStopRequested() && (visionPortal1.getCameraState() != VisionPortal.CameraState.STREAMING)) {
            sleep(20);
        }
        telemetry.addData("Camera", "Ready");
        telemetry.update();
    }

    // Set camera controls unless we are stopping.
        if (!isStopRequested())
    {
        ExposureControl exposureControl = visionPortal1.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            sleep(50);
        }
        exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
        sleep(20);
        GainControl gainControl = visionPortal1.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        sleep(20);
    }
}


    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {

            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;
            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            telemetry.addData("Distance to Pixle: ", getDistance(recognition.getWidth()));
            telemetry.addData("Angle to object: ", getAngleTarget(middleOfObject(recognition.getWidth(), recognition.getLeft())));

        }   // end for() loop

    }

    private static double getDistance(double width) {
        return (objectWidthInRealWorldUnits * focalLength) / width;// end method telemetryTfod()
    }

    private static double getAngleTarget(double objMidpoint) {
        return -((objMidpoint - (CAMERA_WIDTH / 2)) * FOV) / CAMERA_WIDTH;
    }

    public static float middleOfObject(float width, float left) {
        return left + (width / 2);
    }
    public void setDegrees(double deg){
        this.degrees = deg;
    }
    public double getDegrees(){
        return this.degrees;
    }
    public void setisStraight(boolean deg){
        this.isStraight = deg;
    }
}


