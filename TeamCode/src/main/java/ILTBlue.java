
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by 299876 on 12/14/2017.
 */




@Autonomous(name="ILT Blue", group="V")
public class ILTBlue extends LinearOpMode {
    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    public DcMotor rightM;
    public DcMotor leftM;
    public DcMotor rightC;
    public DcMotor leftC;
    public DcMotor liftT;
    public DcMotor liftB;
    public Servo rightH;
    public ColorSensor rightS;
    int i = 0;

    //Creates variables for distances used later
    double r = 41.5;
    double l = 26.5;
    double c = 34;

    // calculates the number of encoder tics for every inch the robot moves
    private double TICKS_PER_IN = 1120/(4*Math.PI);

    //function that takes in a double for inches, left power, and right power
    public void driveDistance(double inches, double leftPower, double rightPower)
    {

        leftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*create two integers that use the current motor
        position, the inches given, and the calculation from motor
        tics to inches in order to tell the motor how far to go for each side*/
        int leftTickGoal = leftM.getCurrentPosition () + (int) (TICKS_PER_IN * inches);
        int rightTickGoal = rightM.getCurrentPosition() + (int) (TICKS_PER_IN * inches);

        /*sets the motors to the new goal created above*/
        leftM.setTargetPosition(leftTickGoal);
        rightM.setTargetPosition(rightTickGoal);

        /*tells the motors to continue moving until the new goal is acheived*/
        while( Math.abs(leftM.getCurrentPosition()) < leftTickGoal || Math.abs(rightM.getCurrentPosition()) < rightTickGoal)
        {

            //tells motors to go to the position set above
            leftM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //sets left and right motors to power given
            rightM.setPower(rightPower/2);//posss -1
            leftM.setPower(leftPower/2);//poss +1

            telemetry.addData("Left Enc ", leftM.getCurrentPosition());
            telemetry.addData("Right Enc ", rightM.getCurrentPosition());
            updateTelemetry(telemetry);
            telemetry.update();
        }

        //tell all motors to stop once the required distance is reached
        leftM.setPower(0);
        rightM.setPower(0);

    }



    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {

        //tells the program what the devices are named in the hardware map on the phone
        rightM = hardwareMap.dcMotor.get("rightM");
        leftM = hardwareMap.dcMotor.get("leftM");
        rightC = hardwareMap.dcMotor.get("rightC");
        leftC = hardwareMap.dcMotor.get("leftC");
        liftT = hardwareMap.dcMotor.get("liftT");
        liftB = hardwareMap.dcMotor.get("liftB");
        rightH = hardwareMap.servo.get("rightH");
        rightS = hardwareMap.colorSensor.get("rightS");

		/*tell jewel servo to be up in order to stay inside the cnstraints during initialization*/
        rightH.setPosition(.1);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AcaIHID/////AAAAGfxtLCdtek2StO17ue0Qv58vOtoKhEUMbPtKsKEn2j6O+r8N8aHlIv8rlPkvXTUfpjbpjashfAtx/NZhQJemzLtk9BjxwlW5VGYaUbOpiLxz8L0yxh8owF8XbCf79eLaUmD7J8BpMgyNKPijZJWl2r7uwc+Mmum8S0cnwliFN2/Pt8y6aYd0eEppco4tMn/WrkQ+GspPLUZ7CGRYh2goHG14ZSuVOXEDx7poPvGgYm9A5pCzuSNSE1dl0uppOAJaw307cZ6YIKX3CZgw92+XquVm0fvfHrLcR1Cj09R56vZKoEZH4S+uWaTofEwlGK7QRp5Kz2CNn8S8W5E80COVniyNPjIRi5WWjl6qDSY6nyrH";

        //chooses camera to use to search for the pictograph
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        //loads pictographs to match the picture on the phone
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        leftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //reverses the left motors so that positive power makes it go forward
        leftM.setDirection(DcMotorSimple.Direction.REVERSE);

        //sets encoder values to zero
        leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //transitions to initialized driver program
        org.firstinspires.ftc.teamcode.AutoTransition.transitionOnStop(this, "TeleOp");

        waitForStart();

        relicTrackables.activate();

        rightC.setPower(-.5);
        leftC.setPower(.5);
        sleep(500);
        rightC.setPower(.3);
        leftC.setPower(.3);

        //repeats the program a maximum of 2 times
        while (opModeIsActive() && i < 3) {

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

            //attempts to match pictograph
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                rightM.setDirection(DcMotorSimple.Direction.FORWARD);
                leftM.setDirection(DcMotorSimple.Direction.REVERSE);

                //displays whether it an see a pictograph
                telemetry.addData("VuMark", "%s visible", vuMark);

                //displays which pictograph is visible on the driver station
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                // moves if it detects a pictograph pose
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot controller
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot controller
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }

                    //slowly puts down color senseor for the jewels

                    rightH.setPosition(1);

                    sleep(1000);

                    //if it detects red, it moves forward to double check
                    if (rightS.red() > rightS.blue())
                    {

                        leftM.setDirection(DcMotorSimple.Direction.REVERSE);
                        rightM.setDirection(DcMotorSimple.Direction.FORWARD);

				/* considering light is naturally red, the
				robot moves forward to detect again if it sees red in order to prevent moving the
				incorrect color*/
                        driveDistance(1.5,.25,.25);

						/*removes one inch from the measured distance to compensate for the inch that it moved forward*/
                        r -= 1.5;
                        l -= 1.5;
                        c -= 1.5;

                        //stops all drive motors
                        leftM.setPower(0);
                        rightM.setPower(0);
                        sleep(1000);


                        //if it reads red again, it rotates counterclockwise
                        if (rightS.red() > rightS.blue()) {


                            // reverses the right motors so that positive numbers rotate the robot counterclockwise

                            rightM.setDirection(DcMotorSimple.Direction.REVERSE);
                            leftM.setDirection(DcMotorSimple.Direction.REVERSE);


                            //moves both sides to their encoder goals at 1/4 power
                            driveDistance(3, .25, .25);

                            //stops all drive motors
                            leftM.setPower(0);
                            rightM.setPower(0);
                            sleep(500);

                            //sets encoders back to 0

                            leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            rightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



                            //set both sides forwad to rotate the robot clockwise to readjust straight

                            rightM.setDirection(DcMotorSimple.Direction.FORWARD);
                            leftM.setDirection(DcMotorSimple.Direction.FORWARD);


                            //puts up color senor
                            rightH.setPosition(.1);


                            //stops all drive motors
                            leftM.setPower(0);
                            rightM.setPower(0);
                            sleep(500);


                            //moves both sides to their encoder goals at 1/4 power
                            driveDistance(3, .25, .25);


                            //sets encoders back to 0

                            leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            rightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                            //stops all drive motors
                            leftM.setPower(0);
                            rightM.setPower(0);
                            sleep(500);
                        }

                        //if it detects blue rotate clockwise

                        else if (rightS.red() < rightS.blue()) {


                            // sets left motors forward so that positive numbers rotate the robot clockwise

                            leftM.setDirection(DcMotorSimple.Direction.FORWARD);


                            //moves both sides to their encoder goals at 1/4 power
                            driveDistance(3, .25, .25);
                            leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            rightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                            //stops all drive motors
                            leftM.setPower(0);
                            rightM.setPower(0);
                            sleep(500);


                            //puts up color senor
                            rightH.setPosition(.1);

                            //stops all drive motors
                            leftM.setPower(0);
                            rightM.setPower(0);
                            sleep(500);


                            //set both sides reverse to rotate the robot counterclockwise
                            leftM.setDirection(DcMotorSimple.Direction.REVERSE);
                            rightM.setDirection(DcMotorSimple.Direction.REVERSE);


                            //moves both sides to their encoder goals at 1/4 power
                            driveDistance(3, .25, .25);


                            //sets encoders back to 0
                            leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            rightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                            //stops all drive motors
                            leftM.setPower(0);
                            rightM.setPower(0);
                            sleep(500);
                        }
                    }


                    //if it detects blue rotate clockwise
                    else if (rightS.red() < rightS.blue())
                    {


                        // sets left motors forward so that positive numbers rotate the robot clockwise

                        leftM.setDirection(DcMotorSimple.Direction.FORWARD);


                        //moves both sides to their encoder goals at 1/4 power
                        driveDistance(3,.25,.25);


                        //sets encoders back to 0

                        leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                        //puts up color senor
                        rightH.setPosition(.1);

                        //stops all drive motors
                        leftM.setPower(0);
                        rightM.setPower(0);
                        sleep(500);


                        //set both sides reverse to rotate the robot counterclockwise

                        leftM.setDirection(DcMotorSimple.Direction.REVERSE);
                        rightM.setDirection(DcMotorSimple.Direction.REVERSE);


                        //moves both sides to their encoder goals at 1/4 power
                        driveDistance(3,.25,.25);


                        //sets encoders back to 0

                        leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                        //stops all drive motors
                        leftM.setPower(0);
                        rightM.setPower(0);
                        sleep(500);
                    }


                    //sets all motors so that positive powers move the robot forward

                    leftM.setDirection(DcMotorSimple.Direction.REVERSE);
                    rightM.setDirection(DcMotorSimple.Direction.FORWARD);

                    /* function that is used if the phone reads that the robot needs to put the glyph
                    in the right side of the cryptobox*/
                    if (vuMark == RelicRecoveryVuMark.RIGHT ){

                        //makes robot move the distance created above at half speed
                        driveDistance(r,.5,.5);

                        //sets all drive motors to zero power
                        leftM.setPower(0);
                        rightM.setPower(0);
                        sleep(100);

                        //sets encoder values to zero
                        leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        //sets direction of both left wheels to foward so the robot can turn toward the cryptobox
                        leftM.setDirection(DcMotorSimple.Direction.FORWARD);

                        //sets robot to half speed and tells it how much distance to move
                        //turns 90
                        driveDistance(13.352, .5,.5);

                        //sets both left wheels to reverse so the robot moves forward
                        leftM.setDirection(DcMotorSimple.Direction.REVERSE);

                        //sets all motors to zero power
                        leftM.setPower(0);
                        rightM.setPower(0);
                        sleep(100);

                        //sets encoder values to zero
                        leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        //sets robot to half speed and tells it how much distance to move
                        driveDistance(12,.5,.5);

                        //sets all motors to zero power
                        leftM.setPower(0);
                        rightM.setPower(0);
                        sleep(100);

                        //opens clamp to drop glyph into the column
                        rightC.setPower(.25);
                        leftC.setPower(-.25);
                        sleep(500);
                        rightC.setPower(0);
                        leftC.setPower(0);

                        rightM.setDirection(DcMotorSimple.Direction.REVERSE);
                        leftM.setDirection(DcMotorSimple.Direction.FORWARD);

                        leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        driveDistance(8, .25, .25);

                        rightM.setDirection(DcMotorSimple.Direction.FORWARD);
                        leftM.setDirection(DcMotorSimple.Direction.REVERSE);

                        leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        driveDistance(11, .25, .25);

                        rightM.setDirection(DcMotorSimple.Direction.REVERSE);
                        leftM.setDirection(DcMotorSimple.Direction.FORWARD);

                        leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        driveDistance(4, .25, .25);

                        requestOpModeStop();
                        stop();
                    }

                    /* function that is used if the phone reads that the robot needs to put the glyph
                    in the center of the cryptobox*/
                    else if (vuMark == RelicRecoveryVuMark.CENTER){

                        driveDistance(c,.5,.5);

                        //sets all motors to zero power
                        leftM.setPower(0);
                        rightM.setPower(0);
                        sleep(100);

                        leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        leftM.setDirection(DcMotorSimple.Direction.FORWARD);

                        driveDistance(13.352, .5,.5);

                        leftM.setDirection(DcMotorSimple.Direction.REVERSE);

                        //sets all motors to zero power
                        leftM.setPower(0);
                        rightM.setPower(0);
                        sleep(100);

                        leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        driveDistance(12,.5,.5);

                        //sets all motors to zero power
                        leftM.setPower(0);
                        rightM.setPower(0);
                        sleep(100);

                        //opens clamp
                        rightC.setPower(.25);
                        leftC.setPower(-.25);
                        sleep(500);
                        rightC.setPower(0);
                        leftC.setPower(0);

                        rightM.setDirection(DcMotorSimple.Direction.REVERSE);
                        leftM.setDirection(DcMotorSimple.Direction.FORWARD);

                        leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        driveDistance(8, .25, .25);

                        rightM.setDirection(DcMotorSimple.Direction.FORWARD);
                        leftM.setDirection(DcMotorSimple.Direction.REVERSE);

                        leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        driveDistance(11, .25, .25);

                        rightM.setDirection(DcMotorSimple.Direction.REVERSE);
                        leftM.setDirection(DcMotorSimple.Direction.FORWARD);

                        leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        driveDistance(4, .25, .25);

                        requestOpModeStop();
                        stop();
                    }
                    else if (vuMark == RelicRecoveryVuMark.LEFT){

                        driveDistance(l,.5,.5);

                        //sets all motors to zero power
                        leftM.setPower(0);
                        rightM.setPower(0);
                        sleep(100);

                        leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        leftM.setDirection(DcMotorSimple.Direction.FORWARD);

                        driveDistance(13.352, .5,.5);

                        leftM.setDirection(DcMotorSimple.Direction.REVERSE);


                        //sets all motors to zero power
                        leftM.setPower(0);
                        rightM.setPower(0);
                        sleep(100);

                        leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        driveDistance(12,.5,.5);

                        //sets all motors to zero power
                        leftM.setPower(0);
                        rightM.setPower(0);
                        sleep(100);

                        //oppen clamps
                        rightC.setPower(.25);
                        leftC.setPower(-.25);
                        sleep(500);
                        rightC.setPower(0);
                        leftC.setPower(0);

                        rightM.setDirection(DcMotorSimple.Direction.REVERSE);
                        leftM.setDirection(DcMotorSimple.Direction.FORWARD);

                        leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        driveDistance(8, .25, .25);

                        rightM.setDirection(DcMotorSimple.Direction.FORWARD);
                        leftM.setDirection(DcMotorSimple.Direction.REVERSE);

                        leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        driveDistance(11, .25, .25);

                        rightM.setDirection(DcMotorSimple.Direction.REVERSE);
                        leftM.setDirection(DcMotorSimple.Direction.FORWARD);

                        leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        driveDistance(4, .25, .25);

                        requestOpModeStop();
                        stop();
                    }

                    relicTrackables.deactivate();


            }                                                                           //end
            else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
        }

        sleep(1000);

        rightM.setDirection(DcMotorSimple.Direction.FORWARD);
        leftM.setDirection(DcMotorSimple.Direction.REVERSE);

        driveDistance(1.5, .25, .25);


        //sets all motors to zero power
        leftM.setPower(0);
        rightM.setPower(0);
        sleep(100);


        rightH.setPosition(1);

        sleep(1000);


        if (rightS.red() > rightS.blue())
        {

            leftM.setDirection(DcMotorSimple.Direction.REVERSE);
            rightM.setDirection(DcMotorSimple.Direction.FORWARD);

				/* considering light is naturally red, the
				robot moves forward to detect again if it sees red in order to prevent moving the
				incorrect color*/
            driveDistance(1.01,.25,.25);

						/*removes one inch from the measured distance to compensate for the inch that it moved forward*/
            r -= 1;
            l -= 1;
            c -= 1;

            //stops all drive motors
            leftM.setPower(0);
            rightM.setPower(0);
            sleep(1000);


            //if it reads red again, it rotates counterclockwise
            if (rightS.red() > rightS.blue()) {


                // reverses the right motors so that positive numbers rotate the robot counterclockwise


                rightM.setDirection(DcMotorSimple.Direction.REVERSE);
                leftM.setDirection(DcMotorSimple.Direction.REVERSE);

                //moves both sides to their encoder goals at 1/4 power
                driveDistance(3, .25, .25);


                //stops all drive motors
                leftM.setPower(0);
                rightM.setPower(0);
                sleep(500);

                //sets encoders back to 0

                leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



                //set both sides forwad to rotate the robot clockwise to readjust straight

                rightM.setDirection(DcMotorSimple.Direction.FORWARD);
                leftM.setDirection(DcMotorSimple.Direction.FORWARD);


                //puts up color senor
                rightH.setPosition(.1);


                //stops all drive motors
                leftM.setPower(0);
                rightM.setPower(0);
                sleep(500);


                //moves both sides to their encoder goals at 1/4 power
                driveDistance(3, .25, .25);


                //sets encoders back to 0

                leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                //stops all drive motors
                leftM.setPower(0);
                rightM.setPower(0);
                sleep(500);
            }

            //if it detects blue rotate clockwise

            else if (rightS.red() < rightS.blue()) {


                // sets left motors forward so that positive numbers rotate the robot clockwise

                leftM.setDirection(DcMotorSimple.Direction.FORWARD);


                //moves both sides to their encoder goals at 1/4 power
                driveDistance(3, .25, .25);
                leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                //stops all drive motors
                leftM.setPower(0);
                rightM.setPower(0);
                sleep(500);


                //puts up color senor
                rightH.setPosition(.1);

                //stops all drive motors
                leftM.setPower(0);
                rightM.setPower(0);
                sleep(500);


                //set both sides reverse to rotate the robot counterclockwise
                leftM.setDirection(DcMotorSimple.Direction.REVERSE);
                rightM.setDirection(DcMotorSimple.Direction.REVERSE);


                //moves both sides to their encoder goals at 1/4 power
                driveDistance(3, .25, .25);


                //sets encoders back to 0
                leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                //stops all drive motors
                leftM.setPower(0);
                rightM.setPower(0);
                sleep(500);
            }
        }


        //if it detects blue rotate clockwise
        else if (rightS.red() < rightS.blue())
        {


            // sets left motors forward so that positive numbers rotate the robot clockwise

            leftM.setDirection(DcMotorSimple.Direction.FORWARD);


            //moves both sides to their encoder goals at 1/4 power
            driveDistance(3,.25,.25);


            //sets encoders back to 0

            leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            //puts up color senor
            rightH.setPosition(.1);

            //stops all drive motors
            leftM.setPower(0);
            rightM.setPower(0);
            sleep(500);


            //set both sides reverse to rotate the robot counterclockwise

            leftM.setDirection(DcMotorSimple.Direction.REVERSE);
            rightM.setDirection(DcMotorSimple.Direction.REVERSE);


            //moves both sides to their encoder goals at 1/4 power
            driveDistance(3,.25,.25);


            //sets encoders back to 0

            leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            //stops all drive motors
            leftM.setPower(0);
            rightM.setPower(0);
            sleep(500);
        }
        leftM.setDirection(DcMotorSimple.Direction.FORWARD);
        rightM.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}