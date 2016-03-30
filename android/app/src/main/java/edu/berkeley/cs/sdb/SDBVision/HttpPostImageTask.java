package edu.berkeley.cs.sdb.SDBVision;

import android.os.AsyncTask;
import android.util.Log;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.text.SimpleDateFormat;
import java.util.Date;

import okhttp3.MediaType;
import okhttp3.MultipartBody;
import okhttp3.OkHttpClient;
import okhttp3.Request;
import okhttp3.RequestBody;
import okhttp3.Response;

public class HttpPostImageTask extends AsyncTask<Void, Void, String> {
    private static final String LOG_TAG = "SDBVision";

    private static final MediaType MEDIA_TYPE_BINARY = MediaType.parse("application/octet-stream");

    private OkHttpClient mHttpClient;
    private String mUrl;
    private byte[] mImageData;
    private int mWidth;
    private int mHeight;
    private Listener mListener;
    private int mSensorOrientation;

    public interface Listener {
        void onResponse(String response);
    }

    public HttpPostImageTask(OkHttpClient httpClient, String url, byte[] imageData, int width, int height, Listener listener, int sensorOrientation) {
        mHttpClient = httpClient;
        mUrl = url;
        mImageData = imageData;
        mWidth = width;
        mHeight = height;
        mListener = listener;
        mSensorOrientation = sensorOrientation;
    }

    /**
     * Rotates YUV Y plane counter-clockwise by 90 degrees.
     * ref: http://stackoverflow.com/a/15775173
     * @param data byte array of Y plane
     * @param imgWidth image width
     * @param imgHeight image height
     * @return Y plane rotated counter-clockwise by 90 degrees
     */
    private byte[] rotateY420Degree90(byte[] data, int imgWidth, int imgHeight) {
        byte[] rotated = new byte[data.length];
        for (int i = 0; i < imgHeight; i++) {
            for (int j = 0; j < imgWidth; j++) {
                rotated[imgHeight * (imgWidth-1 - j) + i] = data[imgWidth * i + j];
            }
        }

        return rotated;
    }

    @Override
    protected String doInBackground(Void... voids) {
        // rotate image counter-clockwise. sensor orientation is clockwise relative to TextureView
        int rotateCount = (mSensorOrientation / 90) % 4;

        for (int i = 0; i < rotateCount; i++) {
            if (i % 2 == 0) {
                mImageData = rotateY420Degree90(mImageData, mWidth, mHeight);
            } else { // need to swap width and height
                mImageData = rotateY420Degree90(mImageData, mHeight, mWidth);
            }
        }

        // send image resolution along with data
        int imageWidth = rotateCount % 2 == 0 ? mWidth : mHeight;
        int imageHeight= rotateCount % 2 == 0 ? mHeight : mWidth;
        // Make sure the byte order is network order (big endian)
        byte[] widthBytes = ByteBuffer.allocate(4).order(ByteOrder.BIG_ENDIAN).putInt(imageWidth).array();
        byte[] heightBytes = ByteBuffer.allocate(4).order(ByteOrder.BIG_ENDIAN).putInt(imageHeight).array();

        String timeStamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
        RequestBody requestBody = new MultipartBody.Builder()
                .setType(MultipartBody.FORM)
                .addFormDataPart("file", timeStamp, RequestBody.create(MEDIA_TYPE_BINARY, mImageData))
                .addFormDataPart("width", timeStamp, RequestBody.create(MEDIA_TYPE_BINARY, widthBytes))
                .addFormDataPart("height", timeStamp, RequestBody.create(MEDIA_TYPE_BINARY, heightBytes))
                .build();
        Request request = new Request.Builder()
                .url(mUrl)
                .post(requestBody)
                .build();

        String result = null;
        try {
            Response response = mHttpClient.newCall(request).execute();
            if (response.isSuccessful()) {
                result = response.body().string();
            } else {
                Log.e(LOG_TAG, "HTTP Error " + response.code() + ":" + response.message());
            }
        } catch (IOException e) {
            e.printStackTrace();
        }

        if (result != null && !result.trim().equals("")) {
            return result.trim();
        }

        return null;
    }

    @Override
    protected void onPostExecute(String response) {
        mListener.onResponse(response);
    }
}
