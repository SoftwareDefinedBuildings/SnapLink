/*
 * Based on code from http://blog.csdn.net/torvalbill/article/details/40378539
 */
package edu.berkeley.cs.sdb.buildingmodel;

import android.app.Activity;
import android.content.Context;
import android.graphics.SurfaceTexture;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CameraMetadata;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.util.Log;
import android.util.Size;
import android.view.Surface;
import android.view.TextureView;
import android.widget.Toast;

import org.apache.http.HttpResponse;
import org.apache.http.HttpStatus;
import org.apache.http.StatusLine;
import org.apache.http.client.HttpClient;
import org.apache.http.client.methods.HttpPost;
import org.apache.http.entity.mime.MultipartEntityBuilder;
import org.apache.http.impl.client.DefaultHttpClient;
import org.apache.http.protocol.BasicHttpContext;
import org.apache.http.protocol.HttpContext;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;


public class MainActivity extends Activity {

    public enum State {INIT, IDLE, STARTING, STOPPING, RUNNING}

    private static final String LOG_TAG = "SDB3D";
    private static final int CAPTURE_IMAGE_REQUEST_CODE = 410;
    private static final String IMAGE_UPLOAD_URL = "http://castle.cs.berkeley.edu:50012/";
    private static final String IMAGE_DIRECTORY = "/storage/emulated/0/DCIM/CAMERA";

    private State mState = State.IDLE;
    private TextureView mTextureView;
    private Handler mHandler;
    private Size mPreviewSize;
    private CameraDevice mCameraDevice;
    private CaptureRequest.Builder mPreviewBuilder;
    private CameraCaptureSession mPreviewSession;
    private HandlerThread mPreviewThread;
    private HttpClient mHttpClient;

    private CameraCaptureSession.StateCallback mPreviewSessionStateCallback = new CameraCaptureSession.StateCallback() {
        @Override
        public void onConfigured(CameraCaptureSession session) {
            if (mState == State.STARTING) {
                mPreviewSession = session;
                updatePreview();
                mState = State.RUNNING;
                Log.i(LOG_TAG, "openCamera finish");
            }
        }

        @Override
        public void onConfigureFailed(CameraCaptureSession session) {
            Toast.makeText(MainActivity.this, "onConfigureFailed", Toast.LENGTH_LONG).show();
        }

        @Override
        public void onReady(CameraCaptureSession session) {
            if (mState == State.STOPPING) {
                mPreviewSession.close();
            }
        }

        @Override
        public void onClosed(CameraCaptureSession session) {
            if (mState == State.STOPPING) {
                mPreviewSession = null;

                if (mPreviewThread != null) {
                    mPreviewThread.quitSafely();
                    mPreviewThread = null;
                }
                mPreviewBuilder = null;
                mPreviewSize = null;
                if (mCameraDevice != null) {
                    mCameraDevice.close();
                }
            }
        }
    };

    private CameraDevice.StateCallback mCameraDeviceStateCallback = new CameraDevice.StateCallback() {
        @Override
        public void onOpened(CameraDevice camera) {
            Log.i(LOG_TAG, "onOpened");
            if (mState == State.STARTING) {
                mCameraDevice = camera;
                startPreview();
            }
        }

        public void onClosed(CameraDevice camera) {
            Log.i(LOG_TAG, "onOpened");
            if (mState == State.STOPPING) {
                Log.i(LOG_TAG, "closeCamera finish");
                mCameraDevice = null;
                mState = State.IDLE;
            }
        }

        @Override
        public void onDisconnected(CameraDevice camera) {
            Log.i(LOG_TAG, "onDisconnected");
        }

        @Override
        public void onError(CameraDevice camera, int error) {
            Log.i(LOG_TAG, "onError (" + String.valueOf(error) + ")");
        }
    };

    private TextureView.SurfaceTextureListener mSurfaceTextureListener = new TextureView.SurfaceTextureListener() {
        @Override
        public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
            Log.i(LOG_TAG, "onSurfaceTextureAvailable, width=" + width + ",height=" + height);
            if (mState == State.INIT || mState == State.IDLE || mState == State.STOPPING) {
                mHandler.post(mStartCamera);
            }
        }

        @Override
        public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {
        }

        @Override
        public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
            return true;
        }

        @Override
        public void onSurfaceTextureUpdated(SurfaceTexture surface) {
        }
    };

    /*
     * Open camera asynchronously
     * Should be called only when mState == State.INIT || mState ==  State.IDLE || mState == State.STOPPING
     */
    private Runnable mStartCamera = new Runnable() {
        @Override
        public void run() {
            if (mState == State.INIT || mState == State.IDLE) {
                mState = State.STARTING;
            } else if (mState == State.STOPPING) {
                mHandler.post(mStartCamera);
                return;
            } else {
                throw new RuntimeException("state error");
            }

            Log.i(LOG_TAG, "openCamera start");
            CameraManager manager = (CameraManager) getSystemService(Context.CAMERA_SERVICE);
            try {
                String cameraId = manager.getCameraIdList()[0];
                CameraCharacteristics characteristics = manager.getCameraCharacteristics(cameraId);
                StreamConfigurationMap map = characteristics.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
                mPreviewSize = map.getOutputSizes(SurfaceTexture.class)[0];

                manager.openCamera(cameraId, mCameraDeviceStateCallback, null);
            } catch (CameraAccessException e) {
                throw new RuntimeException(e);
            }
        }
    };

    /*
     * Close camera asynchronously
     * Should be called only when mState ==  State.STOPPING || mState == State.STARTING
     */
    private Runnable mStopCamera = new Runnable() {
        @Override
        public void run() {
            if (mState == State.RUNNING) {
                mState = State.STOPPING;
            } else if (mState == State.STARTING) {
                mHandler.post(mStopCamera);
                return;
            } else {
                throw new RuntimeException("state error");
            }

            Log.i(LOG_TAG, "closeCamera start");
            if (mPreviewSession != null) {
                try {
                    mPreviewSession.stopRepeating();
                } catch (CameraAccessException e) {
                    throw new RuntimeException(e);
                }

            }
        }

    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);

        mState = State.INIT;

        mTextureView = (TextureView) findViewById(R.id.texture);
        mTextureView.setSurfaceTextureListener(mSurfaceTextureListener);

        mHandler = new Handler();

        mHttpClient = new DefaultHttpClient();
    }

    @Override
    protected void onResume() {
        super.onResume();
        Log.i(LOG_TAG, "onResume");
        if (mState == State.IDLE || mState == State.STOPPING) {
            mHandler.post(mStartCamera);
        }
    }

    @Override
    protected void onPause() {
        super.onPause();
        Log.i(LOG_TAG, "onPause");
        if (mState == State.RUNNING || mState == State.STARTING) {
            mHandler.post(mStopCamera);
        }
    }

    /*
     * Should be called only when mState ==  State.STARTING
     */
    protected void startPreview() {
        if (mState != State.STARTING) {
            throw new RuntimeException("state error");
        }

        if (mCameraDevice == null || !mTextureView.isAvailable() || mPreviewSize == null) {
            Log.i(LOG_TAG, "startPreview fail");
            return;
        }

        SurfaceTexture texture = mTextureView.getSurfaceTexture();
        if (texture == null) {
            Log.i(LOG_TAG, "texture is null");
            throw new RuntimeException("texture is null");
        }

        texture.setDefaultBufferSize(mPreviewSize.getWidth(), mPreviewSize.getHeight());
        Surface surface = new Surface(texture);

        try {
            mPreviewBuilder = mCameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
        } catch (CameraAccessException e) {
            throw new RuntimeException(e);
        }
        mPreviewBuilder.addTarget(surface);

        try {
            mCameraDevice.createCaptureSession(Arrays.asList(surface), mPreviewSessionStateCallback, null);
        } catch (CameraAccessException e) {
            throw new RuntimeException(e);
        }
    }

    /*
     * Should be called only when mState ==  State.STARTING
     * mState should be set to State.RUNNING after calling this method
     */
    protected void updatePreview() {
        if (mState != State.STARTING) {
            throw new RuntimeException("state error");
        }

        if (mCameraDevice == null) {
            throw new RuntimeException("updatePreview error");
        }

        mPreviewBuilder.set(CaptureRequest.CONTROL_MODE, CameraMetadata.CONTROL_MODE_AUTO);
        mPreviewThread = new HandlerThread("CameraPreview");
        mPreviewThread.start();
        Handler backgroundHandler = new Handler(mPreviewThread.getLooper());

        try {
            mPreviewSession.setRepeatingRequest(mPreviewBuilder.build(), null, backgroundHandler);
        } catch (CameraAccessException e) {
            throw new RuntimeException(e);
        }
    }

    /*
     * Uploads captured image to a server and measures the associated delay
     */
    private class UploadImageTask extends AsyncTask<Void, Void, Void> {
        private final HttpClient httpClient;
        private long startTime;
        private long endTime;
        private File imageFile;

        public UploadImageTask(HttpClient httpClient, File imageFile) {
            this.httpClient = httpClient;
            this.imageFile = imageFile;
        }

        @Override
        protected Void doInBackground(Void... voids) {
            startTime = System.currentTimeMillis();

            HttpContext localContext = new BasicHttpContext();
            HttpPost httpPost = new HttpPost(IMAGE_UPLOAD_URL);
            MultipartEntityBuilder multipartEntityBuilder = MultipartEntityBuilder.create();
            multipartEntityBuilder.addBinaryBody("file", imageFile);
            httpPost.setEntity(multipartEntityBuilder.build());

            try {
                Log.i(LOG_TAG, "Starting HTTP Post");
                HttpResponse response = httpClient.execute(httpPost, localContext);
                Log.i(LOG_TAG, "Finished HTTP Post");
                StatusLine statusLine = response.getStatusLine();
                if (statusLine.getStatusCode() != HttpStatus.SC_OK) {
                    String msg = String.format("HTTP Error %d: %s", statusLine.getStatusCode(), statusLine.getReasonPhrase());
                    throw new RuntimeException(msg);
                }
            } catch (IOException e) {
                throw new RuntimeException(e);
            }

            endTime = System.currentTimeMillis();

            return null;
        }

        @Override
        protected void onPostExecute(Void v) {
            String msg = String.format("Image %s posted in %d msec\n", imageFile.getName(), endTime - startTime);
            Log.i(LOG_TAG, msg);
        }
    }
}