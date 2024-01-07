package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Disabled
@Autonomous(name = "April Tag Test")
public class AprilTagTest extends LinearOpMode
{
    private final int RESOLUTION_WIDTH = 1280;
    private final int RESOLUTION_HEIGHT = 720;
    private static final int desiredTagID = -1;
    private static final double BACKDROP_SAFETY_DISTANCE = 12.0; // in inches
    private CameraName camera;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal portal;
    private boolean targetFound;
    private AprilTagDetection desiredTag;


    @Override
    public void runOpMode()
    {
        camera = hardwareMap.get(WebcamName.class, "cool cam");
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        aprilTagProcessor.setDecimation(2);
        portal = new VisionPortal.Builder().setCamera(camera).setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT)).addProcessor(aprilTagProcessor).build();
        portal.stopLiveView();
        targetFound = false;
        desiredTag  = null;
        List<AprilTagDetection> currentDetections = null;
        telemetry.addData("C4", "Ready");
        telemetry.update();
        waitForStart();

        currentDetections = aprilTagProcessor.getDetections();
        sleep(250);
        while (currentDetections == null || currentDetections.size() == 0)
        {
            currentDetections = aprilTagProcessor.getFreshDetections();
            sleep(250);
        }
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        telemetry.update();

        for (AprilTagDetection detection : currentDetections)
        {
            if (detection.metadata != null && !(targetFound))
            {
                if (detection.id == desiredTagID || detection.id >= 0)
                {
                    targetFound = true;
                    desiredTag = detection;
                }
                else
                {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    telemetry.update();
                }
            }
            else if (targetFound) {break;}
            else
            {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                telemetry.update();
            }
        }
        if (targetFound) {
            telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", desiredTag.ftcPose.x, desiredTag.ftcPose.y, desiredTag.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", desiredTag.ftcPose.pitch, desiredTag.ftcPose.roll, desiredTag.ftcPose.yaw));
            double  rangeError      = (desiredTag.ftcPose.range - BACKDROP_SAFETY_DISTANCE);
            double  headingError    = desiredTag.ftcPose.bearing;
            double  yawError        = desiredTag.ftcPose.yaw;
        } else {
            telemetry.addData("\n>","Drive using joysticks to find valid target\n");
            telemetry.update();
        }

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
        telemetry.update();
        while (opModeIsActive()) {sleep(100);}
    }
}