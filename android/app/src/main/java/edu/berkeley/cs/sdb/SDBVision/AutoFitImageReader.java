package edu.berkeley.cs.sdb.SDBVision;

import android.graphics.ImageFormat;
import android.media.Image;
import android.media.ImageReader;
import android.os.Handler;
import android.view.Surface;

import java.nio.ByteBuffer;
import java.util.concurrent.atomic.AtomicBoolean;

public class AutoFitImageReader implements AutoCloseable {

    ImageReader mImageReader;
    OnImageAvailableListener mListener;
    AtomicBoolean mCaptureRequest;

    public interface OnImageAvailableListener {
        void onImageAvailable(byte[] imageData, int width, int height);
    }

    private final ImageReader.OnImageAvailableListener mOnImageAvailableListener = new ImageReader.OnImageAvailableListener() {
        @Override
        public void onImageAvailable(ImageReader reader) {
            Image image;
            try {
                image = reader.acquireLatestImage();
            } catch (IllegalStateException e) {
                e.printStackTrace();
                return;
            }

            // Stop transmitting images after one result comes back
            if (mCaptureRequest.getAndSet(false)) {
                // TODO: do the rotation and scaling here
                byte[] imageData = imageToBytes(image);
                int width = image.getWidth();
                int height = image.getHeight();
                image.close();
                mListener.onImageAvailable(imageData, width, height);
            } else {
                if (image != null) {
                    image.close();
                }
            }
        }
    };

    public AutoFitImageReader(int width, int height, int format, int maxImages) {
        mImageReader = ImageReader.newInstance(width, height, format, maxImages);
        mCaptureRequest = new AtomicBoolean(false);
    }

    public Surface getSurface() {
        return mImageReader.getSurface();
    }

    public void close() {
        if (mImageReader != null) {
            mImageReader.close();
            mImageReader = null;
        }
    }

    public void setOnImageAvailableListener(OnImageAvailableListener listener, Handler handler) {
        mListener = listener;
        mImageReader.setOnImageAvailableListener(mOnImageAvailableListener, handler);
    }

    /**
     * Request the ImageReader to call the listener with the next imediate image.
     * This method is thread safe.
     */
    public void requestCapture() {
        mCaptureRequest.set(true);
    }

    /**
     * Takes an Android Image in the YUV_420_888 format and returns a byte array.
     * ref: http://stackoverflow.com/questions/30510928/convert-android-camera2-api-yuv-420-888-to-rgb
     *
     * @param image Image in the YUV_420_888 format
     * @return bytes that contains the image data in greyscale
     */
    private static byte[] imageToBytes(Image image) {
        if (image.getFormat() != ImageFormat.YUV_420_888) {
            return null;
        }

        int bytesPerPixel = ImageFormat.getBitsPerPixel(ImageFormat.YUV_420_888) / 8;
        Image.Plane yPlane = image.getPlanes()[0]; // we only need a gray picture
        int pixelStride = yPlane.getPixelStride();
        if (bytesPerPixel != 1 || pixelStride != 1) { // they are guaranteed to be both 1 in Y plane
            throw new RuntimeException("Wrong image format");
        }

        ByteBuffer buffer = yPlane.getBuffer();
        int width = image.getWidth();
        int height = image.getHeight();
        byte[] data = new byte[width * height];
        buffer.get(data);

        return data;
    }
}
