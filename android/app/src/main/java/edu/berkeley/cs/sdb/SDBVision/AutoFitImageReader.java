package edu.berkeley.cs.sdb.SDBVision;

import android.media.Image;
import android.media.ImageReader;
import android.os.Handler;
import android.view.Surface;

import java.util.concurrent.atomic.AtomicBoolean;

public class AutoFitImageReader implements AutoCloseable {

    ImageReader mImageReader;
    OnImageAvailableListener mListener;
    AtomicBoolean mCaptureRequest;

    public interface OnImageAvailableListener {
        void onImageAvailable(Image image);
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
                mListener.onImageAvailable(image);
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

    public Surface getSurface () {
        return mImageReader.getSurface();
    }

    public void close (){
        if (mImageReader != null) {
            mImageReader.close();
            mImageReader = null;
        }
    }

    public void setOnImageAvailableListener(OnImageAvailableListener listener, Handler handler) {
        mListener = listener;
        mImageReader.setOnImageAvailableListener(mOnImageAvailableListener, handler);
    }

    public void requestCapture() {
        mCaptureRequest.set(true);
    }
}
