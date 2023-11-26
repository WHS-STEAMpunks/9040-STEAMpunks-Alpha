package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "BlocksRule (Blocks to Java)")
@Disabled
public class BlocksRule extends LinearOpMode {

  private ColorSensor ColorSensorAsColorSensor;
  private DcMotor backleft;
  private DcMotor backright;
  private DcMotor frontleft;
  private DcMotor frontrightAsDcMotor;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    ColorSensorAsColorSensor = hardwareMap.get(ColorSensor.class, "ColorSensorAsColorSensor");
    backleft = hardwareMap.get(DcMotor.class, "back left");
    backright = hardwareMap.get(DcMotor.class, "back right");
    frontleft = hardwareMap.get(DcMotor.class, "front left");
    frontrightAsDcMotor = hardwareMap.get(DcMotor.class, "frontrightAsDcMotor");

    // Put initialization blocks here.
    waitForStart();
    if (0 < ColorSensorAsColorSensor.blue()) {
      backleft.setPower(-0.23);
      backright.setPower(0.25);
      frontleft.setPower(-0.23);
      frontrightAsDcMotor.setPower(0.25);
      sleep(2700);
      frontrightAsDcMotor.setPower(0);
      frontleft.setPower(0);
      backleft.setPower(0);
      backright.setPower(0);
    }
    ColorSensorAsColorSensor.enableLed(true);
    if (false) {
      if (true) {
        return;
      }
    }
    if (ColorSensorAsColorSensor.blue() > ColorSensorAsColorSensor.green() && ColorSensorAsColorSensor.blue() > ColorSensorAsColorSensor.red()) {
      backleft.setPower(0);
      frontleft.setPower(0);
      frontrightAsDcMotor.setPower(0);
      backright.setPower(0);
      telemetry.addData("purple", ColorSensorAsColorSensor.blue());
    } else if (ColorSensorAsColorSensor.green() > ColorSensorAsColorSensor.blue() && ColorSensorAsColorSensor.green() > ColorSensorAsColorSensor.red()) {
      backleft.setPower(-0.25);
      backright.setPower(-0.25);
      frontleft.setPower(0.25);
      frontrightAsDcMotor.setPower(0.25);
      sleep(4000);
      backleft.setPower(0);
      backright.setPower(0);
      frontleft.setPower(0);
      frontrightAsDcMotor.setPower(0);
      telemetry.addData("green", ColorSensorAsColorSensor.green());
    } else {
      if (ColorSensorAsColorSensor.red() > ColorSensorAsColorSensor.blue() && ColorSensorAsColorSensor.red() > ColorSensorAsColorSensor.green()) {
        backleft.setPower(0.25);
        frontleft.setPower(-0.25);
        backright.setPower(0.25);
        frontrightAsDcMotor.setPower(-0.25);
        sleep(4000);
        backleft.setPower(0);
        frontrightAsDcMotor.setPower(0);
        backright.setPower(0);
        frontleft.setPower(0);
        telemetry.addData("orange", ColorSensorAsColorSensor.red());
        backleft.setPower(-0.23);
        frontrightAsDcMotor.setPower(0.23);
        frontleft.setPower(-0.23);
        backright.setPower(0.23);
        sleep(800);
        backleft.setPower(0);
        frontleft.setPower(0);
        frontrightAsDcMotor.setPower(0);
        backright.setPower(0);
      } else {
        telemetry.addData("None Detected", ColorSensorAsColorSensor.red());
      }
    }
    telemetry.update();
  }
}
