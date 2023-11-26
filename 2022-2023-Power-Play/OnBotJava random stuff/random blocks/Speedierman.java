package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Speedierman (Blocks to Java)")
@Disabled
public class Speedierman extends LinearOpMode {

  private DcMotor frontLeftAsDcMotor;
  private DcMotor frontRightAsDcMotor;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    frontLeftAsDcMotor = hardwareMap.get(DcMotor.class, "frontLeftAsDcMotor");
    frontRightAsDcMotor = hardwareMap.get(DcMotor.class, "frontRightAsDcMotor");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        if (gamepad1.a) {
          frontLeftAsDcMotor.setPower(0.9);
        } else {
          frontLeftAsDcMotor.setPower(0);
        }
        if (gamepad1.b) {
          frontRightAsDcMotor.setPower(0.9);
        } else {
          frontRightAsDcMotor.setPower(0);
        }
        // Put loop blocks here.
        telemetry.update();
      }
    }
  }
}
