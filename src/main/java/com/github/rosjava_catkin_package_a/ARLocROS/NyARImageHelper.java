package com.github.rosjava_catkin_package_a.ARLocROS;

import jp.nyatla.nyartoolkit.core.NyARException;
import jp.nyatla.nyartoolkit.core.raster.rgb.INyARRgbRaster;
import jp.nyatla.nyartoolkit.core.raster.rgb.NyARRgbRaster;
import jp.nyatla.nyartoolkit.core.types.NyARBufferType;
import org.opencv.core.Mat;

import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;

/**
 * Helper class to convert and OpenCV Mat containing a camera image to
 * NyARRGBRaster
 *
 */
public class NyARImageHelper extends NyARRgbRaster {

    /**
     * @param image
     * @return
     */
    public static INyARRgbRaster createFromMat(Mat image) {
        BufferedImage bimg;
        if (image != null) {
            int cols = image.cols();
            int rows = image.rows();
            int elemSize = (int) image.elemSize();
            byte[] data = new byte[cols * rows * elemSize];
            int type;
            image.get(0, 0, data);
            // we only support RGB
            type = BufferedImage.TYPE_3BYTE_BGR;
            // bgr to rgb
            byte b;
            for (int i = 0; i < data.length; i = i + 3) {
                b = data[i];
                data[i] = data[i + 2];
                data[i + 2] = b;
            }

            bimg = new BufferedImage(cols, rows, type);

            bimg.getRaster().setDataElements(0, 0, cols, rows, data);
        } else {
            bimg = null;
        }

        NyARImageHelper ra = null;

        int raster_type = NyARBufferType.BYTE1D_B8G8R8_24;
        try {
            ra = new NyARImageHelper(bimg.getWidth(), bimg.getHeight(), raster_type, false);
            ra._buf = ((DataBufferByte) (bimg.getRaster().getDataBuffer())).getData();
            ra._rgb_pixel_driver.switchRaster(ra);
        } catch (NyARException e) {
            e.printStackTrace();
        }

        return ra;

    }

    /**
     * @param i_width
     * @param i_height
     * @param i_raster_type
     * @param i_is_alloc
     * @throws NyARException
     */
    private NyARImageHelper(int i_width, int i_height, int i_raster_type, boolean i_is_alloc) throws NyARException {

        super(i_width, i_height, i_raster_type, i_is_alloc);
    }

}

