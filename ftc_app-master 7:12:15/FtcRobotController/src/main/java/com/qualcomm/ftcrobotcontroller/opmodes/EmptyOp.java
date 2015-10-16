package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

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
    
    
    //Takes in value from controller (-1 to 1) and outputs a value for the motors (-1 to 1)
    double scaleInput(double dVal)  {
		double mag = abs(dVal);
		if(mag < 10) // dead zone
			return 0;
		else
			return signum(dVal)*(mag-.1)*(mag - .1) * 1.2345; //notice: quadratic
	}

    private void printVals(String name, double[] values) {
        for(int i = 0; i < values.length; i++)
            telemetry.addData(name, values[i]);
    }
}
