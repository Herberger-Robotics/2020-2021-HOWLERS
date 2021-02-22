package org.firstinspires.ftc.teamcode.subsystems.Camera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.arcrobotics.ftclib.vision.UGRectRingPipeline;

import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;


public class RingDetector {

    private OpenCvCamera phoneCamera;
    private UGContourRingPipeline pipeline;

    public RingDetector(OpenCvCamera cam) {
        phoneCamera = cam;
        pipeline = new UGContourRingPipeline();

        phoneCamera.openCameraDevice();
        phoneCamera.setPipeline(pipeline);
        phoneCamera.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);

        //FtcDashboard.getInstance().startCameraStream(phoneCamera,0);

        UGContourRingPipeline.Config.setHORIZON(150);
        UGContourRingPipeline.Config.setLowerOrange(new Scalar(100.0,140.0,40.0));
    }



    public UGContourRingPipeline.Height getHeight() {
        return pipeline.getHeight();
    }

    public UGContourRingPipeline getPipeline() {
        return pipeline;
    }
}
