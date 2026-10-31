package org.firstinspires.ftc.robotcontroller.external.samples;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "ThriftestCam_AprilTag_Detector", group = "Vision")
public class ConceptAprilTag extends LinearOpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        // 1. Configure AprilTag Processor with unique ThriftestCam Lens Intrinsics
        // These constants are paired explicitly with the 640x480 resolution view.
        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                .build();

        // 2. Initialize the Vision Portal matching ThriftestCam's operational profile
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // Must match your hardware config name
                .setCameraResolution(new Size(640, 480))                  // Calibration fails if resolution is altered
                .addProcessor(aprilTag)
                .build();

        telemetry.addData("Status", "ThriftestCam Configured. Waiting for Start...");
        telemetry.update();

        // Wait for the driver to press PLAY
        waitForStart();

        while (opModeIsActive()) {
            // 3. Retrieve current frame target detections
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            telemetry.addData("# AprilTags Detected", currentDetections.size());

            // 4. Iterate and display calibrated localization variables
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) Target Verified ====", detection.id));

                    // 3D Cartesian coordinates relative to ThriftestCam lens center point (inches)
                    telemetry.addData("X (Left/Right)", "%6.2f in", detection.ftcPose.x);
                    telemetry.addData("Y (Forward/Back)", "%6.2f in", detection.ftcPose.y);
                    telemetry.addData("Z (Up/Down)", "%6.2f in", detection.ftcPose.z);

                    // Directional and orientation angles (degrees)
                    telemetry.addData("Yaw (Twist Angle)", "%6.2f deg", detection.ftcPose.yaw);
                    telemetry.addData("Pitch (Tilt Angle)", "%6.2f deg", detection.ftcPose.pitch);
                    telemetry.addData("Roll (Roll Angle)", "%6.2f deg", detection.ftcPose.roll);

                    // Spatial Vector values (great for tracking Math formulas)
                    telemetry.addData("Range (Distance)", "%6.2f in", detection.ftcPose.range);
                    telemetry.addData("Bearing (Angle)", "%6.2f deg", detection.ftcPose.bearing);
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Missing Metadata ====", detection.id));
                    telemetry.addLine("Tag detected, but physical size parameters are unknown.");
                }
            }

            telemetry.update();
            sleep(20); // Small interval rest to preserve processing bandwidth
        }

        // 5. Clean close down of camera streaming components
        visionPortal.close();
    }
}
