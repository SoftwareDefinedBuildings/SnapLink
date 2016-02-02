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

    private static final String LOG_TAG = "SDB3D";
    private static final int CAPTURE_IMAGE_REQUEST_CODE = 410;
    private static final String IMAGE_UPLOAD_URL = "http://castle.cs.berkeley.edu:50012/";
    private static final String IMAGE_DIRECTORY = "/storage/emulated/0/DCIM/CAMERA";


    private TextureView mTextureView;
    private Size mPreviewSize;
    private CameraDevice mCameraDevice;
    private CaptureRequest.Builder mPreviewBuilder;
    private CameraCaptureSession mPreviewSession;
    private HttpClient mHttpClient;
    private CameraCaptureSession.StateCallback mCameraCaptureSessionStateCallback = new CameraCaptureSession.StateCallback() {
        @Override
        public void onConfigured(CameraCaptureSession session) {
            mPreviewSession = session;
            updatePreview();
        }

        @Override
        public void onConfigureFailed(CameraCaptureSession session) {
            Toast.makeText(MainActivity.this, "onConfigureFailed", Toast.LENGTH_LONG).show();
        }
    };
    private CameraDevice.StateCallback mCameraDeviceStateCallback = new CameraDevice.StateCallback() {
        @Override
        public void onOpened(CameraDevice camera) {
            Log.e(LOG_TAG, "onOpened");
            mCameraDevice = camera;
            startPreview();
        }

        @Override
        public void onDisconnected(CameraDevice camera) {
            Log.e(LOG_TAG, "onDisconnected");
        }

        @Override
        public void onError(CameraDevice camera, int error) {
            Log.e(LOG_TAG, "onError (" + String.valueOf(error) + ")");
        }
    };
    private TextureView.SurfaceTextureListener mSurfaceTextureListener = new TextureView.SurfaceTextureListener() {
        @Override
        public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
            Log.e(LOG_TAG, "onSurfaceTextureAvailable, width=" + width + ",height=" + height);
            openCamera();
        }

        @Override
        public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {
            Log.e(LOG_TAG, "onSurfaceTextureSizeChanged");
        }

        @Override
        public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
            return false;
        }

        @Override
        public void onSurfaceTextureUpdated(SurfaceTexture surface) {
            Log.e(LOG_TAG, "onSurfaceTextureUpdated");
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);

        mTextureView = (TextureView) findViewById(R.id.texture);
        mTextureView.setSurfaceTextureListener(mSurfaceTextureListener);

        mHttpClient = new DefaultHttpClient();
    }

    @Override
    protected void onResume() {
        super.onResume();
        Log.e(LOG_TAG, "onResume");
    }

    @Override
    protected void onPause() {
        Log.e(LOG_TAG, "onPause");
        super.onPause();
        if (mCameraDevice != null) {
            mCameraDevice.close();
            mCameraDevice = null;
        }
    }

    private void openCamera() {
        CameraManager manager = (CameraManager) getSystemService(Context.CAMERA_SERVICE);
        Log.e(LOG_TAG, "openCamera E");
        try {
            String cameraId = manager.getCameraIdList()[0];
            CameraCharacteristics characteristics = manager.getCameraCharacteristics(cameraId);
            StreamConfigurationMap map = characteristics.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
            mPreviewSize = map.getOutputSizes(SurfaceTexture.class)[0];

            manager.openCamera(cameraId, mCameraDeviceStateCallback, null);
        } catch (CameraAccessException e) {
            throw new RuntimeException(e);
        }
        Log.e(LOG_TAG, "openCamera X");
    }

    protected void startPreview() {
        if (mCameraDevice == null || !mTextureView.isAvailable() || mPreviewSize == null) {
            Log.e(LOG_TAG, "startPreview fail");
            throw new RuntimeException("startPreview fail");
        }

        SurfaceTexture texture = mTextureView.getSurfaceTexture();
        if (texture == null) {
            Log.e(LOG_TAG, "texture is null");
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
            mCameraDevice.createCaptureSession(Arrays.asList(surface), mCameraCaptureSessionStateCallback, null);
        } catch (CameraAccessException e) {
            throw new RuntimeException(e);
        }
    }

    protected void updatePreview() {
        if (mCameraDevice == null) {
            throw new RuntimeException("updatePreview error");
        }

        mPreviewBuilder.set(CaptureRequest.CONTROL_MODE, CameraMetadata.CONTROL_MODE_AUTO);
        HandlerThread thread = new HandlerThread("CameraPreview");
        thread.start();
        Handler backgroundHandler = new Handler(thread.getLooper());

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
                Log.d(LOG_TAG, "Starting HTTP Post");
                HttpResponse response = httpClient.execute(httpPost, localContext);
                Log.d(LOG_TAG, "Finished HTTP Post");
                StatusLine statusLine = response.getStatusLine();
                if (statusLine.getStatusCode() != HttpStatus.SC_OK) {
                    String msg = String.format("HTTP Error %d: %s", statusLine.getStatusCode(),
                            statusLine.getReasonPhrase());
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