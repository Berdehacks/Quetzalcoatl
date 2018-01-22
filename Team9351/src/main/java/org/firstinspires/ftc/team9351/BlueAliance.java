/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.team9351;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 *
 * This is an example LinearOpMode that shows how to use
 * the Adafruit RGB Sensor.  It assumes that the I2C
 * cable for the sensor is connected to an I2C port on the
 * Core Device Interface Module.
 *
 * It also assuems that the LED pin of the sensor is connected
 * to the digital signal pin of a digital port on the
 * Core Device Interface Module.
 *
 * You can use the digital port to turn the sensor's onboard
 * LED on or off.
 *
 * The op mode assumes that the Core Device Interface Module
 * is configured with a name of "dim" and that the Adafruit color sensor
 * is configured as an I2C device with a name of "sensor_color".
 *
 * It also assumes that the LED pin of the RGB sensor
 * is connected to the signal pin of digital port #5 (zero indexed)
 * of the Core Device Interface Module.
 *
 * You can use the X button on gamepad1 to toggle the LED on and off.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name = "Blue Alliance", group = "Sensor")
@Disabled                            // Comment this out to add to the opmode list
public class BlueAliance extends LinearOpMode {

  ColorSensor sensorRGB;
  DeviceInterfaceModule cdim;

  // we assume that the LED pin of the RGB sensor is connected to
  // digital port 5 (zero indexed).
  static final int LED_CHANNEL = 5;

  HardwareOmni         robot   = new HardwareOmni();   // Use a Pushbot's hardware
  HardwareCosas cosas = new HardwareCosas();

  private ElapsedTime runtime = new ElapsedTime();

  static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: Andymark Motor Encoder
  static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
  static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
  static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
          (WHEEL_DIAMETER_INCHES * 3.1415);
  static final double     DRIVE_SPEED             = 0.6;
  static final double     TURN_SPEED              = 0.5;

  @Override
  public void runOpMode() {
    robot.init(hardwareMap);
    cosas.init(hardwareMap);

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F,0F,0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // get a reference to the RelativeLayout so we can change the background
    // color of the Robot Controller app to match the hue detected by the RGB sensor.
    int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
    final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

    // bPrevState and bCurrState represent the previous and current state of the button.
    boolean bPrevState = false;
    boolean bCurrState = false;

    // bLedOn represents the state of the LED.
    boolean bLedOn = true;

    // get a reference to our DeviceInterfaceModule object.
    cdim = hardwareMap.deviceInterfaceModule.get("dim");

    // set the digital channel to output mode.
    // remember, the Adafruit sensor is actually two devices.
    // It's an I2C sensor and it's also an LED that can be turned on or off.
    cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannel.Mode.OUTPUT);

    // get a reference to our ColorSensor object.
    sensorRGB = hardwareMap.colorSensor.get("sensor_color");

    // turn the LED on in the beginning, just so user will know that the sensor is active.
    cdim.setDigitalChannelState(LED_CHANNEL, bLedOn);

    robot.init(hardwareMap);

    // Send telemetry message to signify robot waiting;
    telemetry.addData("Status", "Resetting Encoders");    //
    telemetry.update();

    robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    // Send telemetry message to indicate successful Encoder reset
    telemetry.addData("Path0",  "Starting at FL:%7d FR:%7d BL:%7d BR:%7d",
            robot.frontLeftDrive.getCurrentPosition(),
            robot.frontRightDrive.getCurrentPosition(),
            robot.backLeftDrive.getCurrentPosition(),
            robot.backRightDrive.getCurrentPosition());

    telemetry.update();

    // wait for the start button to be pressed.
    waitForStart();

    // loop and read the RGB data.
    // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
    while (opModeIsActive())  {


      telemetry.addData("Path", "Complete");
      telemetry.update();
      // check the status of the x button on gamepad.
      bCurrState = gamepad1.x;

      // check for button-press state transitions.
      if ((bCurrState == true) && (bCurrState != bPrevState))  {

        // button is transitioning to a pressed state. Toggle the LED.
        bLedOn = !bLedOn;
        cdim.setDigitalChannelState(LED_CHANNEL, bLedOn);
      }

      // update previous state variable.
      bPrevState = bCurrState;

      // convert the RGB values to HSV values.
      Color.RGBToHSV((sensorRGB.red() * 255) / 800, (sensorRGB.green() * 255) / 800, (sensorRGB.blue() * 255) / 800, hsvValues);

      // send the info back to driver station using telemetry function.
      telemetry.addData("LED", bLedOn ? "On" : "Off");
      telemetry.addData("Clear", sensorRGB.alpha());
      telemetry.addData("Red  ", sensorRGB.red());
      telemetry.addData("Green", sensorRGB.green());
      telemetry.addData("Blue ", sensorRGB.blue());
      telemetry.addData("Hue", hsvValues[0]);


      // change the background color to match the color detected by the RGB sensor.
      // pass a reference to the hue, saturation, and value array as an argument
      // to the HSVToColor method.
      relativeLayout.post(new Runnable() {
        public void run() {
          relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
        }
      });

      if (sensorRGB.alpha() > 100){

      }
      telemetry.update();
    }

    // Set the panel back to the default color
    relativeLayout.post(new Runnable() {
      public void run() {
        relativeLayout.setBackgroundColor(Color.WHITE);
      }
    });
  }
  public void encoderDrive(double speed,
                           double frontLeftInches, double frontRightInches,
                           double backLeftInches, double backRightInches,
                           double timeoutS) {
    int newFrontLeftTarget;
    int newFrontRightTarget;
    int newBackLeftTarget;
    int newBackRightTarget;

    // Ensure that the opmode is still active
    if (opModeIsActive()) {

      // Determine new target position, and pass to motor controller
      newFrontLeftTarget = robot.frontLeftDrive.getCurrentPosition() + (int)(frontLeftInches * COUNTS_PER_INCH);
      newFrontRightTarget = robot.frontRightDrive.getCurrentPosition() + (int)(frontRightInches * COUNTS_PER_INCH);
      newBackLeftTarget = robot.backLeftDrive.getCurrentPosition() + (int)(backLeftInches * COUNTS_PER_INCH);
      newBackRightTarget = robot.backRightDrive.getCurrentPosition() + (int)(backRightInches * COUNTS_PER_INCH);

      robot.frontLeftDrive.setTargetPosition(newFrontLeftTarget);
      robot.frontRightDrive.setTargetPosition(newFrontRightTarget);
      robot.backLeftDrive.setTargetPosition(newBackLeftTarget);
      robot.backRightDrive.setTargetPosition(newBackRightTarget);

      // Turn On RUN_TO_POSITION
      robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

      // reset the timeout time and start motion.
      runtime.reset();
      robot.frontLeftDrive.setPower(Math.abs(speed));
      robot.frontRightDrive.setPower(Math.abs(speed));
      robot.backLeftDrive.setPower(Math.abs(speed));
      robot.backRightDrive.setPower(Math.abs(speed));

      // keep looping while we are still active, and there is time left, and both motors are running.
      // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
      // its target position, the motion will stop.  This is "safer" in the event that the robot will
      // always end the motion as soon as possible.
      // However, if you require that BOTH motors have finished their moves before the robot continues
      // onto the next step, use (isBusy() || isBusy()) in the loop test.
      while (opModeIsActive() &&
              (runtime.seconds() < timeoutS) &&
              (robot.frontLeftDrive.isBusy() && robot.frontRightDrive.isBusy() && robot.backLeftDrive.isBusy() && robot.backRightDrive.isBusy())) {

        // Display it for the driver.
        telemetry.addData("Path1",  "Running to FL:%7d FR:%7d BL:%7d BR:%7d", newFrontLeftTarget,  newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
        telemetry.addData("Path2",  "Running at FL:%7d FR:%7d BL:%7d BR: %7d",
                robot.frontLeftDrive.getCurrentPosition(),
                robot.frontRightDrive.getCurrentPosition(),
                robot.backLeftDrive.getCurrentPosition(),
                robot.backRightDrive.getCurrentPosition());
        telemetry.update();
      }

      // Stop all motion;
      robot.frontLeftDrive.setPower(0);
      robot.frontRightDrive.setPower(0);
      robot.backLeftDrive.setPower(0);
      robot.backRightDrive.setPower(0);

      // Turn off RUN_TO_POSITION
      robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

      //  sleep(250);   // optional pause after each move
    }
  }
}
