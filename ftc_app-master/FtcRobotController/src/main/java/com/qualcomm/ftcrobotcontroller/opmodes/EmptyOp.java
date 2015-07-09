package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * An empty op mode serving as a template for custom OpModes
 */

// Version 1 Jasmine Lee 7/2/15:
// Added method called printVals(String name, double[] values) to display values on the Driver Station

public class EmptyOp extends OpMode {

    /*
    * Constructor
    */
    public EmptyOp() {

    }

    /*
    * Code to run when the op mode is first enabled goes here
    * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
    */
    @Override
    public void start() {

    }


    /*
    * This method will be called repeatedly in a loop
    * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
    */
    @Override
    public void loop() {

    }

    /*
    * Code to run when the op mode is first disabled goes here
    * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
    */
    @Override
    public void stop() {

    }

    private void printVals(String name, double[] values) {
        for(int i = 0; i < values.length; i++)
            telemetry.addData(name, values[i]);
    }
}
