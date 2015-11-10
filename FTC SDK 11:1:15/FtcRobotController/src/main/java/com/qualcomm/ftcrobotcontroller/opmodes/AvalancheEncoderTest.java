package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by JasmineLee on 11/1/15.
 */
public class AvalancheEncoderTest extends AvalancheRobot1
{
    int v_state;

    public AvalancheEncoderTest()
    {
    }

    @Override
    public void init()
    {
        super.init();
        resetDriveEncoders();
    }

    @Override
    public void loop() {

        switch (v_state)
        {
            // Synchronize the state machine and hardware.
            case 0:
                // Reset the encoders to ensure they are at a known good value.
                resetDriveEncoders();

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
                runWithEncoders ();

                //
                // Start the drive wheel motors at full power.
                //
                setDrivePower (1.0f, 1.0f, 1.0f, 1.0f);

                //
                // Have the motor shafts turned the required amount?
                //
                // If they haven't, then the op-mode remains in this state (i.e this
                // block will be executed the next time this method is called).
                //
                if (haveDriveEncodersReached (2880, 2880))
                {
                    //
                    // Reset the encoders to ensure they are at a known good value.
                    //
                    resetDriveEncoders ();

                    //
                    // Stop the motors.
                    //
                    setDrivePower (0.0f, 0.0f, 0.0f, 0.0f);

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
                if (haveDriveEncodersReset ())
                {
                    v_state++;
                }
                break;
            //
            // Turn left until the encoders exceed the specified values.
            //
            case 3:
                runWithEncoders ();
                setDrivePower (-1.0f, -1.0f, 1.0f, 1.0f);
                if (haveDriveEncodersReached (2880, 2880))
                {
                    resetDriveEncoders ();
                    setDrivePower (0.0f, 0.0f, 0.0f, 0.0f);
                    v_state++;
                }
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
                setDrivePower(1.0f, 1.0f, -1.0f, -1.0f);
                if(haveDriveEncodersReached (2880, 2880))
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
}