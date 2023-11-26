package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "BlueDuck (Blocks to Java)")
public class BlueDuck extends LinearOpMode {

  private DcMotor frontleft;
  private DcMotor frontrightAsDcMotor;
  private DcMotor backright;
  private DcMotor backleft;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    frontleft = hardwareMap.get(DcMotor.class, "front left");
    frontrightAsDcMotor = hardwareMap.get(DcMotor.class, "frontrightAsDcMotor");
    backright = hardwareMap.get(DcMotor.class, "back right");
    backleft = hardwareMap.get(DcMotor.class, "back left");

    // Put initialization blocks here.
    waitForStart();
    frontleft.setPower(-0.5);
    frontrightAsDcMotor.setPower(-0.5);
    backright.setPower(0.5);
    backleft.setPower(0.5);
    sleep(1200);
    frontleft.setPower(0);
    frontrightAsDcMotor.setPower(0);
    backleft.setPower(0);
    backright.setPower(0);
    sleep(0);
  }
}
