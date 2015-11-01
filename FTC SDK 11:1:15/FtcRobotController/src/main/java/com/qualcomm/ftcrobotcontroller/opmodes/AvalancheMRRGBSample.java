package com.qualcomm.ftcrobotcontroller.opmodes;

import android.app.Activity;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by JasmineLee on 10/11/15.
 */
public class AvalancheMRRGBSample extends LinearOpMode {

    ColorSensor MRRGBSide;
    ColorSensor MRRGBBottom;
    boolean MRRGBLedSide;
    boolean MRRGBLedBottom;
    DcMotor motorRight;
    DcMotor motorLeft;
    boolean foundBeacon = false;
    boolean pressButton = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // write some device information (connection info, name and type)
        // to the log file.
        hardwareMap.logDevices();

        // get a reference to our ColorSensor object.
        MRRGBSide = hardwareMap.colorSensor.get("MRRGBSide");
        MRRGBBottom = hardwareMap.colorSensor.get("MRRGBBottom");

        //get a reference to out ColorSensor object.
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");

        //state of the LED.
        MRRGBLedSide = false;
        MRRGBLedBottom = true;

        // turn the side LED off always, to get better readings.
        MRRGBSide.enableLed(MRRGBLedSide);
        //turn the bottom LED on always to get better readings.
        MRRGBBottom.enableLed(MRRGBLedBottom);

        // wait one cycle.
        waitOneFullHardwareCycle();

        // wait for the start button to be pressed.
        waitForStart();

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float RGBValuesSide[] = {0F, 0F, 0F};
        float RGBValuesBottom[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float valuesSide[] = RGBValuesSide;
        final float valuesBottom[] = RGBValuesBottom;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);

        // while we haven't found the Beacon, keep moving forward
        while (!foundBeacon) {
            // check the status of the x button on either gamepad.
            // bCurrState = gamepad1.x || gamepad2.x;
            set_drive_power(0.8f, -0.8f); //right motor is reversed

            if (MRRGBBottom.red() == 1.0 && MRRGBBottom.green() == 1.0 && MRRGBBottom.blue() == 1.0) {
                set_drive_power(0.0f, 0.0f);
                foundBeacon = true;
                pressButton = true;
            }
            // check for button state transitions.
            /*if (bCurrState == true && bCurrState != bPrevState)  {
                // button is transitioning to a pressed state.

                // print a debug statement.
                DbgLog.msg("MY_DEBUG - x button was pressed!");

                // update previous state variable.
                bPrevState = bCurrState;

            } else if (bCurrState == false && bCurrState != bPrevState)  {
                // button is transitioning to a released state.

                // print a debug statement.
                DbgLog.msg("MY_DEBUG - x button was released!");

                // update previous state variable.
                bPrevState = bCurrState;*/
            // send the bottom RGB Sensor info back to driver station using telemetry function
            telemetry.addData("Side Red  ", MRRGBSide.red());
            telemetry.addData("Side Green ", MRRGBSide.green());
            telemetry.addData("Side Blue ", MRRGBSide.blue());
            telemetry.addData("Bottom Red ", MRRGBBottom.red());
            telemetry.addData("Bottom Green ", MRRGBBottom.green());
            telemetry.addData("Bottom Blue ", MRRGBBottom.blue());

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            /*relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });*/

            // wait a hardware cycle before iterating.
            waitOneFullHardwareCycle();
        }


        while (pressButton) {
            if (MRRGBSide.red() > MRRGBSide.blue()) {
                telemetry.addData("Color", "red");
                pressButton = false;
            } else if (MRRGBSide.blue() > MRRGBSide.red()) {
                telemetry.addData("Color", "blue");
                pressButton = false;
            }
        }
    }

    // a_left_drive_power: Access the left drive motor's power level.
    double a_left_drive_power ()
    {
        double l_return = 0.0;

        if (motorLeft != null)
        {
            l_return = motorLeft.getPower ();
        }

        return l_return;

    }

    // a_right_drive_power: Access the right drive motor's power level.
    double a_right_drive_power ()
    {
        double l_return = 0.0;

        if (motorRight != null)
        {
            l_return = motorRight.getPower ();
        }

        return l_return;

    }

    // set_drive_power: Scale the joystick input using a nonlinear algorithm.
    void set_drive_power (double p_left_power, double p_right_power)

    {
        if (motorLeft!= null)
        {
            motorLeft.setPower (p_left_power);
        }
        if (motorRight != null)
        {
            motorRight.setPower (p_right_power);
        }

    }
}
