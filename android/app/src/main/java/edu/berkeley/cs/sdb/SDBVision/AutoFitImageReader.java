package edu.berkeley.cs.sdb.SDBVision;

import android.app.Activity;
import android.content.Context;
import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.hardware.camera2.CameraCharacteristics;
import android.media.Image;
import android.media.ImageReader;
import android.os.Handler;
import android.view.Surface;

import java.nio.ByteBuffer;
import java.util.concurrent.atomic.AtomicBoolean;

public class AutoFitImageReader implements AutoCloseable {

    CameraCharacteristics mCharacteristics;
    Context mContext;
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

            if (image == null) {
                return;
            }

            // Stop transmitting images after one result comes back
            if (mCaptureRequest.getAndSet(false)) {
                // TODO: do the rotation and scaling here, this is a worker thread
                int width = image.getWidth();
                int height = image.getHeight();
                byte[] imageData = imageToBytes(image);
                image.close();

                if (isBlurred(imageData, width, height)) {
                    return;
                }

                // image from camera is flipped along x-axis
                imageData = flipVertical(imageData, width, height);

                // scale image to half size if 16:9 ratio
                if (width / 16 * 9 == height || height / 16 * 9 == width) {
                    imageData = halfImage(imageData, width, height);
                    width /= 2;
                    height /= 2;
                }

                //imageData = scale(imageData, width, height);
                imageData = rotate(imageData, width, height);

                if (getRotateCount() % 2 == 0) {
                    mListener.onImageAvailable(imageData, width, height);
                } else {
                    mListener.onImageAvailable(imageData, height, width);
                }
            } else {
                image.close();
            }
        }
    };

    public AutoFitImageReader(Context context, CameraCharacteristics characteristics, int width, int height, int format, int maxImages) {
        mContext = context;
        mCharacteristics = characteristics;

        // TODO this is a temporary hack. Should force view to be in 4:3 so we won't have to change ImageReader aspect ratio
        Rect sensorSize = characteristics.get(CameraCharacteristics.SENSOR_INFO_ACTIVE_ARRAY_SIZE);
        if (sensorSize.width() / 9 * 16 == sensorSize.height() || sensorSize.height() / 9 * 16 == sensorSize.width()) {
            // hardcoded common 16:9 resolution
            mImageReader = ImageReader.newInstance(1280, 720, format, maxImages);
        } else {
            mImageReader = ImageReader.newInstance(width, height, format, maxImages);
        }

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
     * Returns the number of times image has to be rotated clockwise to be in user perspective.
     * @return number of times image has to be rotated 90 degrees clockwise.
     */
    private int getRotateCount() {
        int sensorRotation = mCharacteristics.get(CameraCharacteristics.SENSOR_ORIENTATION) / 90;
        int userRotation = ((Activity) mContext).getWindowManager().getDefaultDisplay().getRotation();
        if (sensorRotation - userRotation < 0) {
            return sensorRotation - userRotation + 4; // loop around to positive value
        } else {
            return sensorRotation - userRotation;
        }
    }

    /**
     * Takes an Android Image in the YUV_420_888 format and returns a byte array.
     * ref: http://stackoverflow.com/questions/30510928/convert-android-camera2-api-yuv-420-888-to-rgb
     *
     * @param image Image in the YUV_420_888 format
     * @return bytes that contains the image data in greyscale
     */
    private byte[] imageToBytes(Image image) {
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

    private boolean isBlurred(byte[] image, int width, int height) {
        return false;
    }

    /**
     * Scale an image to half size
     *
     * @param imageData the raw bytes of a greyscale image, every byte is a color sample
     * @return imageData scaled down to half size
     */
    private byte[] halfImage(byte[] imageData, int width, int height) {
        byte[] halved = new byte[imageData.length / 4];
        for (int i = 0; i < height / 2; i++) {
            for (int j = 0; j < width / 2; j++) {
                halved[width /2 * i + j] = imageData[width * 2 * i + 2 * j];
            }
        }
        return halved;
    }

    /**
     * Scale the image to have the same aspect ratio as the camera sensor
     *
     * @param imageData the raw bytes of a greyscale image, every byte is a color sample
     * @return the raw bytes of the scaled image, every byte is a color sample
     */
    private byte[] scale(byte[] imageData, int width, int height) {
        // TODO read characteristics.get(CameraCharacteristics.SENSOR_INFO_ACTIVE_ARRAY_SIZE)
        return imageData;
    }

    /**
     * Flip the image vertically
     *
     * @param imageData the raw bytes of a greyscale image, every byte is a color sample
     * @return the raw bytes of the image, flipped vertically
     */
    private byte[] flipVertical(byte[] imageData, int width, int height) {
        byte[] flipped = new byte[imageData.length];
        for (int i = 0; i < height; i++) {
            System.arraycopy(imageData, width * i, flipped, width * (height - 1 - i), width);
        }
        return flipped;
    }

    /**
     * Rotate image to the user-inspecting orientation
     *
     * @param imageData the raw bytes of a greyscale image, every byte is a color sample
     * @return the raw bytes of the rotated image, every byte is a color sample
     */
    private byte[] rotate(byte[] imageData, int width, int height) {
        switch (getRotateCount()) {
            case 1:
                imageData = rotate90(imageData, width, height);
                break;
            case 2:
                imageData = rotate180(imageData, width, height);
                break;
            case 3:
                imageData = rotate270(imageData, width, height);
                break;
        }
        return imageData;
    }

    private byte[] rotate90(byte[] imageData, int width, int height) {
        byte[] rotated = new byte[imageData.length];
        for (int i = 0; i < height; i++) {
            for (int j  = 0; j < width; j++) {
                rotated[height * j + (height - 1 - i)] = imageData[width * i + j];
            }
        }
        return rotated;
    }

    private byte[] rotate180(byte[] imageData, int width, int height) {
        byte[] rotated = new byte[imageData.length];
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                rotated[width * (height - 1 - i) + (width - 1 - j)] = imageData[width * i + j];
            }
        }
        return rotated;
    }

    private byte[] rotate270(byte[] imageData, int width, int height) {
        byte[] rotated = new byte[imageData.length];
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                rotated[height * (width - 1 - j) + i] = imageData[width * i + j];
            }
        }
        return rotated;
    }
}