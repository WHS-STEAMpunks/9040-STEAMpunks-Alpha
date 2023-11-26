package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Elmochub (Blocks to Java)")
public class Elmochub extends LinearOpMode {

  private DcMotor frontrightAsDcMotor;
  private DcMotor backright;
  private DcMotor backleft;
  private DcMotor frontleft;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    frontrightAsDcMotor = hardwareMap.get(DcMotor.class, "frontrightAsDcMotor");
    backright = hardwareMap.get(DcMotor.class, "back right");
    backleft = hardwareMap.get(DcMotor.class, "back left");
    frontleft = hardwareMap.get(DcMotor.class, "front left");

    waitForStart();
    frontrightAsDcMotor.setPower(0.5);
    backright.setPower(-0.5);
    backleft.setPower(-0.5);
    frontleft.setPower(0.5);
    sleep(1500);
    frontrightAsDcMotor.setPower(0);
    backright.setPower(0);
    frontleft.setPower(0);
    backleft.setPower(0);
    sleep(0);
  }
}
