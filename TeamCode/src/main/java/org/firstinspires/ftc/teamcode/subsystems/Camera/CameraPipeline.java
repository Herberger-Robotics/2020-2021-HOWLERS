package org.firstinspires.ftc.teamcode.subsystems.Camera;

/**
 * This is an example of vision using EasyOpenCv.This example will show
 * how to convert color spaces,draw rectangles, and extract the value
 * inside the rectangle.
 * For full docs: https://opencv-java-tutorials.readthedocs.io/
 * For the library: https://github.com/OpenFTC/EasyOpenCV
 */

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/**
 * This is the pipeline for our vision system,a pipeline
 * is a distinct steps to process and analyze the input(camera frames)
 */
public class CameraPipeline extends OpenCvPipeline {


    // These are the mats we need, I will be explaining them as we go
    private Mat mat = new Mat();
    private enum Height {
        ZERO, ONE, FOUR,
    }
    private Height height;

    //These will be the points for our rectangle
    int[] left_rect = {
            1,
            2,
            3,
            4
    };

    int[] right_rect = {
            1,
            2,
            3,
            4
    };

    /**
     * This will create the rectangles
     * @param frame the input mat
     * @param points the points for the rectangle
     * @param color the color of the rectangle when it is displayed on screen
     * @param thickness the thickness of the rectangle
     */
    public Mat drawRectangle(Mat frame,int[] points,Scalar color,int thickness){

        Imgproc.rectangle(
                frame,
                new Point(
                        points[0],
                        points[1]),

                new Point(
                        points[2],
                        points[3]),
                color, thickness);

        //submat simply put is cropping the mat
        return frame.submat(points[1], points[3], points[0], points[2]);

    }



    @Override
    public Mat processFrame(Mat input) {

        Scalar lowerOrange = new Scalar(0.0, 141.0, 0.0);
        Scalar upperOrange = new Scalar(255.0, 230.0, 95.0);

        /** width of the camera in use, defaulted to 320 as that is most common in examples **/
        int CAMERA_WIDTH = 320;

        /** Horizon value in use, anything above this value (less than the value) since
         * (0, 0) is the top left of the camera frame **/
        int HORIZON = (int)((100.0 / 320.0) * CAMERA_WIDTH);

        /** algorithmically calculated minimum width for width check based on camera width **/
        int MIN_WIDTH = (int)(50.0 / 320.0) * CAMERA_WIDTH;

        /** if the calculated aspect ratio is greater then this, height is 4, otherwise its 1 **/
        double BOUND_RATIO = 0.7;

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);

        // variable to store mask in
        Mat mask = new Mat(mat.rows(), mat.cols(), CvType.CV_8UC1);
        Core.inRange(mat, lowerOrange, upperOrange, mask);

        Imgproc.GaussianBlur(mask, mask, new Size(5.0, 15.0), 0.00);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

        int maxWidth = 0;
        Rect maxRect = new Rect();
        for (MatOfPoint c : contours) {
            MatOfPoint2f copy = new MatOfPoint2f(c.toArray());
            Rect rect = Imgproc.boundingRect(copy);

            int w = rect.width;
            // checking if the rectangle is below the horizon
            if (w > maxWidth && rect.y + rect.height > HORIZON) {
                maxWidth = w;
                maxRect = rect;
            }
            c.release(); // releasing the buffer of the contour, since after use, it is no longer needed
            copy.release(); // releasing the buffer of the copy of the contour, since after use, it is no longer needed
        }

        //double aspectRatio = maxRect.getHeight() / maxRect.getWidth();
        height = Height.ZERO;
        /*
        // equivalent
        if (maxWidth >= MIN_WIDTH) {
            if (aspectRatio > BOUND_RATIO) {
                height = Height.FOUR;
            } else {
                height = Height.ONE;
            }
        } else {
            height = Height.ZERO;
        }

         */
        return new Mat();

    }


}