package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;import java.lang.Math;import java.lang.Override;

public class AvalancheTeleOp extends OpMode {

    final static double MOTOR_POWER = 0.50; // Higher values will cause the robot to move faster

    final static double HOLD_IR_SIGNAL_STRENGTH = 0.20; // Higher values will cause the robot to follow closer

    DcMotor motorRightFront;
    DcMotor motorRightBack;
    DcMotor motorLeftFront;
    DcMotor motorLeftBack;
    DcMotor motorArm;
    DcMotor motorTape;
   // DcMotor harvesterMotor;
    Servo servoTape;
    Servo scoopTop;
    Servo scoopLeft;
    Servo scoopRight;
    Servo harvestLeft;
    Servo harvestRight;
    Servo tapeAngle;

    public AvalancheTeleOp() {

    }

    @Override
    public void init() {
        //irSeeker = hardwareMap.irSeekerSensor.get("ir_seeker");
        motorRightFront = hardwareMap.dcMotor.get("rf");
        motorRightBack = hardwareMap.dcMotor.get("rb");
        motorLeftFront = hardwareMap.dcMotor.get("lf");
        motorLeftBack = hardwareMap.dcMotor.get("lb");
        motorArm = hardwareMap.dcMotor.get("arm");
        motorTape = hardwareMap.dcMotor.get("tm");
        scoopTop = hardwareMap.servo.get("st");
        scoopLeft = hardwareMap.servo.get("sl");
        scoopRight = hardwareMap.servo.get("sr");
        harvestLeft = hardwareMap.servo.get("hl");
        harvestRight = hardwareMap.servo.get("hr");
        tapeAngle = hardwareMap.servo.get("sa");
      //  harvesterMotor = hardwareMap.dcMotor.get("hm");

        motorRightFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightBack.setDirection(DcMotor.Direction.REVERSE);

        resetDriveEncoders();
        resetArmEncoders();
    }

    private double stValue = 0.0;
    private double slValue = 0.0; //need to figure out starting val
    private double srValue = 0.0; //need to figure out starting val
    private double hlValue = 0.0; //need to figure out starting val
    private double hrValue = 0.0; //need to figure out starting val
    private double tapeValue = 0.0;
    boolean e = true;

    private int armTarget = 0;
    private int integration = 0;
    private long lastTime = 0;
    private int lastPosition = 0;

    boolean releasedX = true;
    boolean releasedY = true;

    //Main Joystick
    /* Driving with joysticks*/
    //Auxiiary Joystick
    /*
        Right Joy: Angle of hook (working on - IDEA)
        Left Joy: Arm Up & Down
        Right Trigger: Harvester In
        Left Trigger: Harvester Out
        Top Hat Top: Hook Out
        Top Hat Bottom: Hook In
        Button A: Complicated Subroutine of servos with scoop (working on - JL)
        Button B: Climbers
        Button Y: Open & Close Scoop (done - JL)
        Button X: harvester toggle
    */

    @Override
    public void loop() {

        long time = System.currentTimeMillis();
        long dt = time - lastTime;
        lastTime = time;

        if(dt > 1000)
            dt = 1;

        runWithEncoders();
        // Joy: left_stick_y ranges from -1 to 1, where 1 is full up, and
        // -1 is full down
        double leftJoy = scaleInput(-gamepad1.left_stick_y);
        double rightJoy = scaleInput(-gamepad1.right_stick_y);

        //driving
        if(gamepad1.right_trigger > .1 || gamepad1.right_bumper || gamepad1.left_trigger > .1|| gamepad1.left_bumper)
        {
            if(gamepad1.right_trigger > .1)
                setDrivePower(1.0,1.0,1.0,1.0);
            else if(gamepad1.right_bumper)
                setDrivePower(.5,.5,.5,.5);
            else if(gamepad1.left_trigger > .1)
                setDrivePower(-1.0,-1.0,-1.0,-1.0);
            else
                setDrivePower(-.5,-.5,-.5,-.5);
        }
        else {
            // write the values to the motors
            motorLeftFront.setPower(leftJoy);
            motorLeftBack.setPower(leftJoy);
            motorRightFront.setPower(rightJoy);
            motorRightBack.setPower(rightJoy);
        }

        //Complicated Subroutine of servos with scoop (complete)
        if(gamepad1.a)
        {
            //if arm is down in harvesting position, move up to horizontal
            if(armEncoderCount() <= 100) //need to test this value to see where harvester needs to move
            {
                boolean armClear = false; //if arm has cleared harvester yet

                //move arm up until harvester needs to move & move harvester
                setArmPower(0.5);
                if(hasArmEncoderReached(100))
                {
                    setArmPower(0.0); //stop arm movement

                    //move harvester up
                    hlValue = 0.45; //need to test values of these
                    hrValue = 0.45;
                    harvestLeft.setPosition(hlValue);
                    harvestRight.setPosition(hrValue);

                    //probably want a slight delay in here

                    //move scoop head up
                    slValue = 0.9;
                    srValue = 0.1;
                    scoopLeft.setPosition(slValue);
                    scoopRight.setPosition(srValue);

                    armClear = true;
                }
                if(armClear)
                {
                    //move arm up to horizontal
                    setArmPower(0.5);

                    if(hasArmEncoderReached(200))
                    {
                        setArmPower(0.0);

                        //move harvester down
                        hlValue = 0.7; //need to test values of these
                        hrValue = 0.3;
                        harvestLeft.setPosition(hlValue);
                        harvestRight.setPosition(hrValue);
                    }
                    armClear = false;
                }
            }
            //if arm is up anywhere, move down to harvesting position
            else
            {
                boolean armClear = false; //if arm has cleared harvester yet
                boolean harvestPos = false; //if arm is in harvest position
                //move arm up until harvester needs to move & move harvester
                setArmPower(-0.5);
                if(hasArmEncoderReached(100))
                {
                    setArmPower(0.0); //stop arm movement

                    //move harvester up
                    hlValue = 0.45; //need to test values of these
                    hrValue = 0.45;
                    harvestLeft.setPosition(hlValue);
                    harvestRight.setPosition(hrValue);

                    //probably want a slight delay in here

                    //move scoop head up
                    slValue = 0.9;
                    srValue = 0.1;
                    scoopLeft.setPosition(slValue);
                    scoopRight.setPosition(srValue);

                    setArmPower(-0.5); //move arm down to harvest position
                    armClear = true;
                }
                if(hasArmEncoderReached(0)) //stop when reached harvest
                {
                    setArmPower(0.0);

                    //move scoop head up
                    slValue = 0.0;
                    srValue = 1.0;
                    scoopLeft.setPosition(slValue);
                    scoopRight.setPosition(srValue);

                    //move harvester down
                    hlValue = 0.7; //need to test values of these
                    hrValue = 0.3;
                    harvestLeft.setPosition(hlValue);
                    harvestRight.setPosition(hrValue);
                }

            }
        }

        //opening & closing scoop
        if(gamepad1.y && releasedY)
        {
            if(stValue == 0.9)
                stValue = 0.0;
            else if(stValue == 0.0)
                stValue = 0.9;
            releasedY = false;
        }
        else
            releasedY = true;

        //values for left and right servos on scoop (needs to be integrated in)
        /*if(slValue > 0.5) {
            slValue = 0.0;
            srValue = 1.0;
        }
        else if(slValue <= 0.5) {
            slValue = 0.9;
            srValue = 0.1;
        }*/

	    scoopTop.setPosition(stValue);

        armTarget += scaleInput(-gamepad2.left_stick_y) * dt/3;

        int pos = motorArm.getCurrentPosition();
        int p = armTarget - pos;
        integration += p * dt;
        long d = (lastPosition - pos)/dt;
        lastPosition = pos;

        double power = .01 * p + .00001 * integration + .001 * d;
        setArmPower(power);


        tapeValue += scaleInput(-gamepad2.right_stick_y) * dt/10;
        tapeAngle.setPosition(tapeValue);

        if(gamepad2.dpad_up)
            motorTape.setPower(.78);
        else if(gamepad2.dpad_down)
            motorTape.setPower(-.78);
        else
            motorTape.setPower(0);

        if(gamepad2.x && releasedX){
            hlValue = hlValue == 0? .45: 0;
            hrValue = hrValue == 0? .45: 0;
            releasedX = false;
        }
        else
            releasedX = true;

       /* if(gamepad2.right_bumper)
            harvesterMotor.setPower(.78);
        else if(gamepad2.left_bumper)
            harvesterMotor.setPower(-.78);
        else
            harvesterMotor.setPower(0);*/
        
		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        telemetry.addData("Text", "*** Robot Data ***");
        //telemetry.addData("left power: ", leftJoy);
        //telemetry.addData("right power: ", rightJoy);
        telemetry.addData("gamepad1.y: ", gamepad1.y);
        telemetry.addData("stVal: ", stValue);
    }
    //}

    @Override
    public void stop() {

    }

    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0)
            index = -index;
        if (index > 16)
            index = 16;


        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }

    void setDrivePower(double LFPower, double LBPower, double RFPower, double RBPower)
    {
        if(motorLeftFront != null)
            motorLeftFront.setPower(LFPower * 0.78);
        else
            throw new RuntimeException("motorLeftFront is null");
        if(motorLeftBack!= null)
            motorLeftBack.setPower(LBPower * 0.78);
        else
            throw new RuntimeException("motorLeftBack is null");
        if(motorRightFront != null)
            motorRightFront.setPower(RFPower * 0.78);
        else
            throw new RuntimeException("motorRightFront is null");
        if(motorRightBack != null)
            motorRightBack.setPower(RBPower * 0.78);
        else
            throw new RuntimeException("motorRightBack is null");
    }

    void setArmPower(double armPower)
    {
    	armPower = Math.max(-1, Math.min(1, armPower));
        armPower = Math.max(-1, Math.min(1, armPower));
        if(motorArm != null)
            motorArm.setPower(armPower * 0.78);
        else
            throw new RuntimeException("motorArm is null");
    }

    public void runWithLeftDriveEncoder()
    {
        if(motorLeftFront != null)
        {
            motorLeftFront.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            motorLeftBack.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }
    }

    // Set the right drive wheel encoder to run, if the mode is appropriate.
    public void runWithRightDriveEncoder()
    {
        if(motorRightFront != null)
        {
            motorRightFront.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            motorRightBack.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }
    }

    // Set both drive wheel encoders to run, if the mode is appropriate.
    public void runWithEncoders()
    {
        // Call other members to perform the action on both motors.
        runWithLeftDriveEncoder();
        runWithRightDriveEncoder();
        motorArm.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
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

    public void resetArmEncoders()
    {
        if(motorArm != null)
        {
            motorArm.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
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

    int armEncoderCount()
    {
        int l_return = 0;

        if(motorArm != null)
        {
            l_return = motorArm.getCurrentPosition ();
        }

        return l_return;
    }

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

    boolean hasArmEncoderReached(double p_count)
    {
        // Assume failure.
        boolean l_return = false;

        if(motorArm != null)
        {
            // Have the encoders reached the specified values?
            // TODO Implement stall code using these variables.
            if(Math.abs(motorArm.getCurrentPosition ()) > p_count)
            {
                // Set the status to a positive indication.
                l_return = true;
            }
        }

        // Return the status.
        return l_return;
    }

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

    //Indicate whether the drive motors' encoders have reached a value.
    boolean driveUsingEncoders( double LFPower, double LBPower, double RFPower, double RBPower,
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

    boolean haveArmEncodersReset()
    {
        // Assume failure.
        boolean l_return = false;

        // Has the right encoder reached zero?
        if (armEncoderCount() == 0)
        {
            // Set the status to a positive indication.
            l_return = true;
        }

        // Return the status.
        return l_return;
    }
}