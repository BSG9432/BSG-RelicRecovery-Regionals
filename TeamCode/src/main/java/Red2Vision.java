import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.AutoTransition;

import static org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark.*;

@Autonomous(name="Red 2 VUFORIA", group = "z")
@Disabled
public class Red2Vision extends LinearOpMode {

    public DcMotor rightM;
    public DcMotor leftM;
    public DcMotor rightC;
    public DcMotor leftC;
    public DcMotor liftT;
    public DcMotor liftB;
    public Servo rightH;
    public ColorSensor rightS;
    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 8.0;
    static final double WHEEL_DIAMETER_INCHES = 3.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    // static final double ticks = 560;

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {

        rightM = hardwareMap.dcMotor.get("rightM");
        leftM = hardwareMap.dcMotor.get("leftM");
        rightC = hardwareMap.dcMotor.get("rightC");
        leftC = hardwareMap.dcMotor.get("leftC");
        liftT = hardwareMap.dcMotor.get("liftT");
        liftB = hardwareMap.dcMotor.get("liftB");
        rightH = hardwareMap.servo.get("rightH");
        rightS = hardwareMap.colorSensor.get("rightS");

        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        boolean bPrevState = false;
        boolean bCurrState = false;
        boolean bLedOn = true;


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        //https://developer.vuforia.com/license-manager

        parameters.vuforiaLicenseKey = "AcaIHID/////AAAAGfxtLCdtek2StO17ue0Qv58vOtoKhEUMbPtKsKEn2j6O+r8N8aHlIv8rlPkvXTUfpjbpjashfAtx/NZhQJemzLtk9BjxwlW5VGYaUbOpiLxz8L0yxh8owF8XbCf79eLaUmD7J8BpMgyNKPijZJWl2r7uwc+Mmum8S0cnwliFN2/Pt8y6aYd0eEppco4tMn/WrkQ+GspPLUZ7CGRYh2goHG14ZSuVOXEDx7poPvGgYm9A5pCzuSNSE1dl0uppOAJaw307cZ6YIKX3CZgw92+XquVm0fvfHrLcR1Cj09R56vZKoEZH4S+uWaTofEwlGK7QRp5Kz2CNn8S8W5E80COVniyNPjIRi5WWjl6qDSY6nyrH\n";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        rightC.setPower(-.1);
        leftC.setPower(.1);

        AutoTransition.transitionOnStop(this, "TeleOp");

        waitForStart();

        telemetry.update();

        sleep(1000);

        rightH.setPosition(1);
        rightH.setPosition(.9);

        sleep(2000);

        telemetry.update();

        telemetry.addData("RIGHT Clear", rightS.alpha());
        telemetry.addData("RIGHT Red  ", rightS.red());
        telemetry.addData("RIGHT Green", rightS.green());
        telemetry.addData("RIGHT Blue ", rightS.blue());
        telemetry.addData("RIGHT Hue", hsvValues[0]);

        telemetry.update();

        if (rightS.blue() > rightS.red()) {

            telemetry.update();

            rightM.setPower(-.25);
            sleep(600);

            rightM.setPower(0);
            sleep(500);

            rightH.setPosition(.1);

            rightM.setPower(0);
            sleep(500);

            rightM.setPower(.25);
            sleep(400);

            rightM.setPower(0);
            sleep(500);

            rightM.setPower(-1);
            leftM.setPower(1);
            sleep(500);

        } else {

            telemetry.update();

            rightM.setPower(.25);
            sleep(600);

            rightM.setPower(0);
            sleep(500);

            rightH.setPosition(.1);

            rightM.setPower(0);
            sleep(500);

            rightM.setPower(-.5);
            sleep(500);

            rightM.setPower(0);
            sleep(500);
        }

        relicTrackables.activate();

        while (opModeIsActive()) {

            RelicRecoveryVuMark vuMark = from(relicTemplate);
            if (vuMark != UNKNOWN) {

                telemetry.addData("VuMark", "%s visible", vuMark);

                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }

                telemetry.update();
            } else {
                telemetry.addData("VuMark", "not visible");
            }

            leftM.setPower(.25);
            rightM.setPower(.25);
            sleep(750);

            switch (vuMark)
            {
                case LEFT:                                                                          //LEFT

                    rightM.setPower(0);
                    leftM.setPower(0);
                    sleep(1000);

                    rightM.setPower(-.25);
                    leftM.setPower(.25);
                    sleep(1000);

                    rightM.setPower(0);
                    leftM.setPower(0);
                    sleep(1000);

                    rightM.setPower(-.5);
                    leftM.setPower(-.5);
                    sleep(800);

                    rightM.setPower(0);
                    leftM.setPower(0);
                    sleep(1000);

                    rightM.setPower(.25);
                    leftM.setPower(-.25);
                    sleep(2000);

                    rightC.setPower(.25);
                    leftC.setPower(-.25);
                    sleep(1000);

                    rightM.setPower(-.25);
                    leftM.setPower(.25);
                    sleep(250);

                    rightC.setPower(-.25);
                    leftC.setPower(.25);
                    sleep(1000);

                    rightM.setPower(.25);
                    leftM.setPower(-.25);
                    sleep(3000);

                    rightM.setPower(-.25);
                    leftM.setPower(.25);
                    sleep(500);

                    telemetry.update();

                    break;

                case CENTER:                                                                        //CENTER

                    rightM.setPower(0);
                    leftM.setPower(0);
                    sleep(1000);

                    rightM.setPower(-.25);
                    leftM.setPower(.25);
                    sleep(2000);

                    rightM.setPower(0);
                    leftM.setPower(0);
                    sleep(1000);

                    rightM.setPower(-.5);
                    leftM.setPower(-.5);
                    sleep(800);

                    rightM.setPower(0);
                    leftM.setPower(0);
                    sleep(1000);

                    rightM.setPower(.25);
                    leftM.setPower(-.25);
                    sleep(2000);

                    rightC.setPower(.25);
                    leftC.setPower(-.25);
                    sleep(1000);

                    rightM.setPower(-.25);
                    leftM.setPower(.25);
                    sleep(250);

                    rightC.setPower(-.25);
                    leftC.setPower(.25);
                    sleep(1000);

                    rightM.setPower(.25);
                    leftM.setPower(-.25);
                    sleep(3000);

                    rightM.setPower(-.25);
                    leftM.setPower(.25);
                    sleep(500);

                    telemetry.update();

                    break;

                case RIGHT:                                                                         //RIGHT
                    rightM.setPower(0);
                    leftM.setPower(0);
                    sleep(1000);

                    rightM.setPower(-.25);
                    leftM.setPower(.25);
                    sleep(3000);

                    rightM.setPower(0);
                    leftM.setPower(0);
                    sleep(1000);

                    rightM.setPower(-.5);
                    leftM.setPower(-.5);
                    sleep(800);

                    rightM.setPower(0);
                    leftM.setPower(0);
                    sleep(1000);

                    rightM.setPower(.25);
                    leftM.setPower(-.25);
                    sleep(2000);

                    rightC.setPower(.25);
                    leftC.setPower(-.25);
                    sleep(1000);

                    rightM.setPower(-.25);
                    leftM.setPower(.25);
                    sleep(250);

                    rightC.setPower(-.25);
                    leftC.setPower(.25);
                    sleep(1000);

                    rightM.setPower(.25);
                    leftM.setPower(-.25);
                    sleep(3000);

                    rightM.setPower(-.25);
                    leftM.setPower(.25);
                    sleep(500);

                    telemetry.update();

                    break;

                default:                                                                            //DEFAULT
                    //Java code
                    rightM.setPower(0);
                    leftM.setPower(0);
                    rightC.setPower(0);
                    leftC.setPower(0);
                    telemetry.update();
                    break;
            }

            if (vuMark != UNKNOWN){
                relicTrackables.deactivate();
            }

            telemetry.update();
        }
        telemetry.update();
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}


