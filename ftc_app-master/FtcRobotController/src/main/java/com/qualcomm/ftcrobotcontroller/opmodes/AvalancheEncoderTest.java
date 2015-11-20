package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.GyroSensor;
/**
 * Created by JasmineLee on 11/1/15.
 */
public class AvalancheEncoderTest extends OpMode
{
    GyroSensor gyro;

    int v_state;
    DcMotor motorRightFront;
    DcMotor motorRightBack;
    DcMotor motorLeftFront;
    DcMotor motorLeftBack;
    private long integration = 0;
    private long lastTime;
    private long lastSuccess = 0;

    public AvalancheEncoderTest()
    {
    }

    @Override
    public void init()
    {
        gyro = hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();
        while (gyro.isCalibrating())  {
            try{Thread.sleep(50);}catch(Exception e){}
        }
        motorRightFront = hardwareMap.dcMotor.get("rf");
        motorRightBack = hardwareMap.dcMotor.get("rb");
        motorLeftFront = hardwareMap.dcMotor.get("lf");
        motorLeftBack = hardwareMap.dcMotor.get("lb");
        motorRightFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightBack.setDirection(DcMotor.Direction.REVERSE);
        resetDriveEncoders();
    }

    @Override
    public void loop() {

        switch (v_state) {
            // Synchronize the state machine and hardware.
            case 0:
                // Reset the encoders to ensure they are at a known good value.
                resetDriveEncoders();
                lastTime = System.currentTimeMillis();
                // Transition to the next state when this method is called again.
                v_state++;

                break;
            //
            // Drive forward until the encoders exceed the specified values.
            //
            case 1:
                //
                // Tell the system that motor encoders will be used.  This call MUST
                // be in this state and NOT the previous or the encoders will not
                // work.  It doesn't need to be in subsequent states.
                //
                runWithEncoders();

                //
                // Start the drive wheel motors at full power.
                //
                setDrivePower(1.0f, 1.0f, 1.0f, 1.0f);

                //
                // Have the motor shafts turned the required amount?
                //
                // If they haven't, then the op-mode remains in this state (i.e this
                // block will be executed the next time this method is called).
                //
                if (haveDriveEncodersReached(10000, 10000)) {
                    //
                    // Reset the encoders to ensure they are at a known good value.
                    //
                    resetDriveEncoders();
                    gyro.resetZAxisIntegrator();

                    // Stop the motors.
                    //
                    //setDrivePower (0.0f, 0.0f, 0.0f, 0.0f);

                    //
                    // Transition to the next state when this method is called
                    // again.
                    //
                    v_state++;
                }
                break;
            //
            // Wait...
            //
            case 2:

                if (haveDriveEncodersReset()) {
                    v_state++;
                }
                break;
            //
            // Turn left until the encoders exceed the specified values.
            //
            case 3:
                runWithEncoders();
                int p = 180 - gyro.getHeading();
                integration += p * (System.currentTimeMillis() - lastTime);
                int d = -gyro.rawZ();

                double power = .005 * p + .000005 * integration + .01 * d;
                power = Math.min(1, power);
                setDrivePower(power, power, -power,  -power);

                if (Math.abs(180 - gyro.getHeading()) < 3) {
                    if(lastSuccess == 0)
                        lastSuccess = System.currentTimeMillis();
                    else if(System.currentTimeMillis() - lastSuccess > 500) {
                        setDrivePower(0.0, 0.0, 0.0, 0.0);
                        resetDriveEncoders();
                        integration = 0;
                        gyro.resetZAxisIntegrator();
                        v_state++;
                    }
                }
                else
                    lastSuccess = 0;
                break;
            //
            // Wait...
            //
            case 4:
                if (haveDriveEncodersReset ())
                {
                    v_state++;
                }
                break;
            //
            // Turn right until the encoders exceed the specified values.
            //
            case 5:
                runWithEncoders();
                setDrivePower(1.0f, 1.0f, 1.0f, 1.0f);
                if(haveDriveEncodersReached (10000, 10000))
                {
                    resetDriveEncoders ();
                    setDrivePower (0.0f, 0.0f, 0.0f, 0.0f);
                    v_state++;
                }
                break;
            //
            // Wait...
            //
            case 6:
                if (haveDriveEncodersReset ())
                {
                    v_state++;
                }
                break;
            //
            // Perform no action - stay in this case until the OpMode is stopped.
            // This method will still be called regardless of the state machine.
            //
            default:
                //
                // The autonomous actions have been accomplished (i.e. the state has
                // transitioned into its final state.
                //
                break;
        }

		/*Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.*/
        telemetry.addData("Text", "*** Robot Data ***");
        telemetry.addData("v_state: ", v_state);
        telemetry.addData("Left Encoder: ", leftEncoderCount());
        telemetry.addData("Right Encoder: ", rightEncoderCount());
    }

    @Override
    public void stop() {

    }

    double scaleInput(double dVal)
    {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0)
            index = -index;
        else if (index > 16)
            index = 16;

        double dScale = 0.0;
        if (dVal < 0)
            dScale = -scaleArray[index];
        else
            dScale = scaleArray[index];

        return dScale;
    }

    void setDrivePower(double LFPower, double LBPower, double RFPower, double RBPower)
    {
        if(motorLeftFront != null)
            motorLeftFront.setPower(LFPower * .78);
        else
            throw new RuntimeException("motorLeftFront is null");
        if(motorLeftBack!= null)
            motorLeftBack.setPower(LBPower * .35);
        else
            throw new RuntimeException("motorLeftBack is null");
        if(motorRightFront != null)
            motorRightFront.setPower(RFPower * .78);
        else
            throw new RuntimeException("motorRightFront is null");
        if(motorRightBack != null)
            motorRightBack.setPower(RBPower * .35);
        else
            throw new RuntimeException("motorRightBack is null");
    }

    public void runWithLeftDriveEncoder()
    {
        if(motorLeftFront != null)
        {
            motorLeftFront.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            motorLeftBack.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }
    }

    // run_using_right_drive_encoder
    // Set the right drive wheel encoder to run, if the mode is appropriate.
    public void runWithRightDriveEncoder()
    {
        if(motorRightFront != null)
        {
            motorRightFront.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            motorRightBack.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }
    }

    // run_using_encoders
    // Set both drive wheel encoders to run, if the mode is appropriate.
    public void runWithEncoders()
    {
        // Call other members to perform the action on both motors.
        runWithLeftDriveEncoder();
        runWithRightDriveEncoder();
    }

    public void resetLeftDriveEncoders()
    {
        if(motorLeftFront != null)
        {
            motorLeftFront.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
            motorLeftBack.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        }
    }

    // Reset the right drive wheel encoder.
    public void resetRightDriveEncoders()
    {
        if(motorRightFront != null)
        {
            motorRightFront.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
            motorRightBack.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        }
    }

    public void resetDriveEncoders()
    {
        resetLeftDriveEncoders();
        resetRightDriveEncoders();
    }

    // Access the left encoder's count.
    int leftEncoderCount()
    {
        int l_return = 0;

        if(motorLeftFront != null)
        {
            l_return = motorLeftFront.getCurrentPosition ();
        }

        return l_return;
    }

    //Access the right encoder's count.
    int rightEncoderCount()
    {
        int l_return = 0;

        if(motorRightFront != null)
        {
            l_return = motorRightFront.getCurrentPosition ();
        }

        return l_return;
    }

    // has_left_drive_encoder_reached
    // Indicate whether the left drive motor's encoder has reached a value.
    boolean hasLeftDriveEncoderReached(double p_count)
    {
        // Assume failure.
        boolean l_return = false;

        if(motorLeftFront != null)
        {
            // Has the encoder reached the specified values?
            // TODO Implement stall code using these variables.
            if(Math.abs(motorLeftFront.getCurrentPosition ()) > p_count)
            {
                // Set the status to a positive indication.
                l_return = true;
            }
        }

        // Return the status.
        return l_return;
    }

    // has_right_drive_encoder_reached
    //Indicate whether the right drive motor's encoder has reached a value.
    boolean hasRightDriveEncoderReached(double p_count)
    {
        // Assume failure.
        boolean l_return = false;

        if(motorRightFront != null)
        {
            // Have the encoders reached the specified values?
            // TODO Implement stall code using these variables.
            if(Math.abs(motorRightFront.getCurrentPosition ()) > p_count)
            {
                // Set the status to a positive indication.
                l_return = true;
            }
        }

        // Return the status.
        return l_return;
    }

    // have_drive_encoders_reached
    //Indicate whether the drive motors' encoders have reached a value.
    boolean haveDriveEncodersReached(double p_left_count, double p_right_count)
    {
        // Assume failure.
        boolean l_return = false;

        // Have the encoders reached the specified values?
        if(hasLeftDriveEncoderReached (p_left_count) &&
                hasRightDriveEncoderReached (p_right_count))
        {
            // Set the status to a positive indication.
            l_return = true;
        }

        // Return the status.
        return l_return;
    }

    // drive_using_encoders
    //Indicate whether the drive motors' encoders have reached a value.
    boolean drive_using_encoders( double LFPower, double LBPower, double RFPower, double RBPower,
                                  double LFCount, double LBCount, double RFCount, double RBCount)
    {
        // Assume the encoders have not reached the limit.
        boolean l_return = false;

        // Tell the system that motor encoders will be used.
        runWithEncoders();

        // Start the drive wheel motors at full power.
        setDrivePower (LFPower, LBPower, RFPower, RBPower);

        // Have the motor shafts turned the required amount?
        // If they haven't, then the op-mode remains in this state (i.e this
        // block will be executed the next time this method is called).
        if (haveDriveEncodersReached (LFCount, LFCount))
        {
            // Reset the encoders to ensure they are at a known good value.
            resetDriveEncoders ();

            // Stop the motors.
            setDrivePower (0.0f, 0.0f, 0.0f, 0.0f);

            // Transition to the next state when this method is called again.
            l_return = true;
        }

        // Return the status.
        return l_return;
    }

    // has_left_drive_encoder_reset
    //Indicate whether the left drive encoder has been completely reset.
    boolean hasLeftDriveEncoderReset()
    {
        // Assume failure.
        boolean l_return = false;

        // Has the left encoder reached zero?
        if (leftEncoderCount() == 0)
        {
            // Set the status to a positive indication.
            l_return = true;
        }

        // Return the status.
        return l_return;
    }

    //has_right_drive_encoder_reset
    //Indicate whether the left drive encoder has been completely reset.
    boolean hasRightDriveEncoderReset()
    {
        // Assume failure.
        boolean l_return = false;

        // Has the right encoder reached zero?
        if (rightEncoderCount() == 0)
        {
            // Set the status to a positive indication.
            l_return = true;
        }

        // Return the status.
        return l_return;
    }

    //have_drive_encoders_reset
    //Indicate whether the encoders have been completely reset.
    boolean haveDriveEncodersReset()
    {
        // Assume failure.
        boolean l_return = false;

        // Have the encoders reached zero?
        if (hasLeftDriveEncoderReset() && hasRightDriveEncoderReset ())
        {
            // Set the status to a positive indication.
            l_return = true;
        }

        // Return the status.
        return l_return;
    }
}
