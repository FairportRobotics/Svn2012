package com.fairportfirst.frc2012.vision;

import com.sun.cldc.jna.BlockingFunction;
import com.sun.cldc.jna.NativeLibrary;
import com.sun.cldc.jna.Pointer;
import com.sun.cldc.jna.TaskExecutor;
import com.sun.cldc.jna.ptr.IntByReference;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.CriteriaCollection;
import edu.wpi.first.wpilibj.image.NIVision;
import edu.wpi.first.wpilibj.image.NIVision.MeasurementType;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.image.ParticleAnalysisReport;

/**
 *
 * @author Tyler the awsome sauce
 */
//extends WPILaptopCameraExtension
public class Camera
{
    CriteriaCollection cc;      // the criteria for doing the particle filter operation
    ParticleAnalysisReport r;
    public int pos;
    int eqRecHeight;
    public double distance;
    ColorImage imageR;
    double scottysConstant = (2*Math.tan(Math.toRadians(23.5)));
    ParticleAnalysisReport[] targets = new ParticleAnalysisReport[4];

    public Camera()
    {
        cc = new CriteriaCollection();      // create the criteria for the particle filter
        cc.addCriteria(MeasurementType.IMAQ_MT_BOUNDING_RECT_WIDTH, 20, 400, false);
        cc.addCriteria(MeasurementType.IMAQ_MT_BOUNDING_RECT_HEIGHT, 30, 400, false);
    }

    public int calImage(ColorImage imag, int placement) throws NIVisionException
    {
        /**
         * Do the image capture with the camera and apply the algorithm described above. This
         * sample will either get images from the camera or from an image file stored in the top
         * level directory in the flash memory on the cRIO. The file name in this case is "10ft2.jpg"
         *
         */
        imageR = imag;
        //ColorImage image = camera.getImage();     // comment if using stored images
        BinaryImage thresholdImage = imag.thresholdHSL(79, 129, 47, 255, 66, 255);   // keep only red objects
        BinaryImage bigObjectsImage = thresholdImage.removeSmallObjects(false, 1);  // remove small artifacts
        BinaryImage convexHullImage = bigObjectsImage.convexHull(false);          // fill in occluded rectangles
        BinaryImage filteredImage = convexHullImage.particleFilter(cc);           // find filled in rectangles
        ParticleAnalysisReport[] reports = filteredImage.getOrderedParticleAnalysisReports();  // get list of results
        System.out.println("CUPCAKES: " + filteredImage.getOrderedParticleAnalysisReports().length);
        for (int i = 0; i < reports.length; i++)
        {
            r = reports[i];
            targets[i] = r;
                        //System.out.println("CUPCAKE YOU AND ME BACK A DA PARK: " +NIVision.countParticles(image.image));
//            eqRecHeight = (int) NIVision.MeasureParticle(image.image,  NIVision.countParticles(image.image), false, MeasurementType.IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE);
            pos = r.center_mass_x;
            System.out.println("Particle: " + placement + ":  Center of mass x: " + r.center_mass_y);
            System.out.println(filteredImage.getNumberParticles());
        }

        /**
         * all images in Java must be freed after they are used since they are allocated out
         * of C data structures. Not calling free() will cause the memory to accumulate over
         * each pass of this loop.
         */
        filteredImage.free();
        convexHullImage.free();
        bigObjectsImage.free();
        thresholdImage.free();
        imag.free();
        try
        {
           return r.center_mass_x;
        }
        catch(NullPointerException e)
        {
            System.out.print("---ERROR---[ Cannot find target ]");
            return 0;
        }

    }
    public double getDistance(int target)
    {
        try
        {
            distance = (7680/r.boundingRectWidth)/scottysConstant;
            System.out.println("Width: " +targets[target].boundingRectWidth);
            return distance;
        }
        catch (ArrayIndexOutOfBoundsException e)
        {
            System.out.println("---ERROR---[ target specified out of range ]");
        }
        catch(NullPointerException e)
        {
            System.out.println("---ERROR--- [ no you are a null pointer exeption ]");
        }
        return 0.0;
    }
}

