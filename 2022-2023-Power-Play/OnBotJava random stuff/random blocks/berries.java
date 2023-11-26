package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@TeleOp(name = "berries (Blocks to Java)")
public class berries extends LinearOpMode {

  private VuforiaCurrentGame vuforiaPOWERPLAY;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    vuforiaPOWERPLAY = new VuforiaCurrentGame();

    waitForStart();
    vuforiaPOWERPLAY.initialize(
        "", // vuforiaLicenseKey
        hardwareMap.get(WebcamName.class, "Cool Cam"), // cameraName
        "", // webcamCalibrationFilename
        false, // useExtendedTracking
        true, // enableCameraMonitoring
        VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
        0, // dx
        0, // dy
        0, // dz
        AxesOrder.XZY, // axesOrder
        90, // firstAngle
        90, // secondAngle
        0, // thirdAngle
        true); // useCompetitionFieldTargetLocations
    sleep(10000);

    vuforiaPOWERPLAY.close();
  }
}
