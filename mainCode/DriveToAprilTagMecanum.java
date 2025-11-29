package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name="Mecanum Drive To AprilTag", group = "Concept")
public class DriveToAprilTagMecanum extends LinearOpMode {

    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 12.0; // inches

    final double SPEED_GAIN  = 0.02;
    final double STRAFE_GAIN = 0.015;
    final double TURN_GAIN   = 0.01;

    final double MAX_AUTO_SPEED  = 0.5;
    final double MAX_AUTO_STRAFE = 0.5;
    final double MAX_AUTO_TURN   = 0.3;

    private DcMotor leftFrontDrive   = null;
    private DcMotor rightFrontDrive  = null;
    private DcMotor leftBackDrive    = null;
    private DcMotor rightBackDrive   = null;

    private static final boolean USE_WEBCAM = true;
    private static final int DESIRED_TAG_ID = -1; // -1 = any tag

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

@Override
public void runOpMode() {

    // Initialize motors and AprilTag processor
    initAprilTag();

    leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
    rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
    leftBackDrive   = hardwareMap.get(DcMotor.class, "backLeft");
    rightBackDrive  = hardwareMap.get(DcMotor.class, "backRight");

    leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
    leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
    rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
    rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

    if (USE_WEBCAM) setManualExposure(6, 250);

    telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
    telemetry.addData(">", "Touch Play to start OpMode");
    telemetry.update();
    waitForStart();

    double drive = 0, strafe = 0, turn = 0;

    while (opModeIsActive()) {

        // Get current detections
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        boolean tagFound = false;
        desiredTag = null;

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && 
                (DESIRED_TAG_ID < 0 || detection.id == DESIRED_TAG_ID)) {
                tagFound = true;
                desiredTag = detection;
                break;
            }
        }

        if (tagFound) {
            // Tag detected → compute errors
            double rangeError   = desiredTag.ftcPose.range - DESIRED_DISTANCE;
            double headingError = desiredTag.ftcPose.bearing;
            double yawError     = desiredTag.ftcPose.yaw;

            // Auto-center and drive toward tag
            drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f", drive, strafe, turn);
        } else {
            // No tag → stay still
            drive = 0;
            strafe = 0;
            turn = 0;

            telemetry.addData("Status", "Waiting for tag...");
        }

        telemetry.update();
        moveRobot(drive, strafe, turn);
        sleep(20);
    }
}

    public void moveRobot(double x, double y, double yaw) {
        double leftFrontPower  = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower   = x + y - yaw;
        double rightBackPower  = x - y + yaw;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    private void initAprilTag() {
        double tagSizeMeters = 0.2032; // 8 inches in meters

        AprilTagLibrary.Builder tagLib = new AprilTagLibrary.Builder();

        // 2025-2026 DECODE official tags
        tagLib.addTag(new AprilTagMetadata(20, "BlueGoal", tagSizeMeters, DistanceUnit.METER));
        tagLib.addTag(new AprilTagMetadata(21, "BlueObeliskA", tagSizeMeters, DistanceUnit.METER));
        tagLib.addTag(new AprilTagMetadata(22, "BlueObeliskB", tagSizeMeters, DistanceUnit.METER));
        tagLib.addTag(new AprilTagMetadata(23, "BlueObeliskC", tagSizeMeters, DistanceUnit.METER));
        tagLib.addTag(new AprilTagMetadata(24, "RedGoal", tagSizeMeters, DistanceUnit.METER));
        tagLib.addTag(new AprilTagMetadata(25, "RedObeliskA", tagSizeMeters, DistanceUnit.METER));
        tagLib.addTag(new AprilTagMetadata(26, "RedObeliskB", tagSizeMeters, DistanceUnit.METER));
        tagLib.addTag(new AprilTagMetadata(27, "RedObeliskC", tagSizeMeters, DistanceUnit.METER));

        // Custom tag 13
        tagLib.addTag(new AprilTagMetadata(13, "MyCustomTag", 0.0065, DistanceUnit.METER));;

        // Build processor
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(tagLib.build())
                .build();

        // Build vision portal
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) return;

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                sleep(20);
            }
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            sleep(50);
        }
        exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
    }
}
