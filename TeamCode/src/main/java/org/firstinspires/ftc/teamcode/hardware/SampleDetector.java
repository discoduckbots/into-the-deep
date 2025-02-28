package org.firstinspires.ftc.teamcode.hardware;
import android.util.Log;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opmode.FancyCancelableTeleop;
import org.firstinspires.ftc.teamcode.opmode.OpencvCancelableTeleop;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class SampleDetector {
    OpenCvCamera webcam;
    private static final String TAG = "ftc-opencv";
    private boolean isDetecting = false;
    private FancyCancelableTeleop.MotorOffset motorOffset = null;
    private Gamepad gamepad = null;

    public SampleDetector(WebcamName webcamName, HardwareMap hardwareMap, FancyCancelableTeleop.MotorOffset motorOffset, Gamepad gamepad) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.motorOffset = motorOffset;
        this.gamepad = gamepad;
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        Log.d(TAG, "open webcam " + webcam);

        webcam.setPipeline(new SamplePipeline());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                Log.d(TAG, "started streaming");

                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                Log.d(TAG, "found an error" + errorCode);

            }
        });
    }

    public void startDetecting() {
        isDetecting = true;
    }

    public void stopDetecting() {
        isDetecting = false;
    }

   class SamplePipeline extends OpenCvPipeline {
        boolean viewportPaused;

        public Mat processFrame(Mat image) {
            if (isDetecting) {
                motorOffset.detected = false;
                OpencvBlockDetector.detectBlock(image, motorOffset);
                if (motorOffset.detected) {
                    // tell teleop to vibrate the controller
                    isDetecting = false;
                    motorOffset.y_offset = 0;
                    motorOffset.x_offset = 0;
                    gamepad.rumble(100);
                }
            }
            return image;
        }
    }

}
