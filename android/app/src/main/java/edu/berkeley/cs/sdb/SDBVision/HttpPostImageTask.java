package edu.berkeley.cs.sdb.SDBVision;

import android.graphics.ImageFormat;
import android.media.Image;
import android.os.AsyncTask;
import android.util.Log;

import java.io.IOException;
import java.nio.ByteBuffer;
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

    private static final MediaType MEDIA_TYPE_JPEG = MediaType.parse("image/jpeg");

    private OkHttpClient mHttpClient;
    private String mUrl;
    private Image mImage;
    private Listener mListener;

    public interface Listener {
        void onResponse(String response);
    }

    public HttpPostImageTask(OkHttpClient httpClient, String url, Image image, Listener listener) {
        mHttpClient = httpClient;
        mUrl = url;
        mImage = image;
        mListener = listener;
    }

    /**
     * Takes an Android Image in the YUV_420_888 format and returns a byte array.
     *
     * @param image Image in the YUV_420_888 format
     * @return bytes that contains the image data
     */
    private static byte[] imagetoBytes(Image image) {
        if (image.getFormat() != ImageFormat.YUV_420_888) {
            return null;
        }

        ByteBuffer buffer;
        int rowStride;
        int pixelStride;
        int width = image.getWidth();
        int height = image.getHeight();
        int offset = 0;

        Image.Plane[] planes = image.getPlanes();
        byte[] data = new byte[image.getWidth() * image.getHeight() * ImageFormat.getBitsPerPixel(ImageFormat.YUV_420_888) / 8];
        byte[] rowData = new byte[planes[0].getRowStride()];

        for (int i = 0; i < planes.length; i++) {
            buffer = planes[i].getBuffer();
            rowStride = planes[i].getRowStride();
            pixelStride = planes[i].getPixelStride();
            int w = (i == 0) ? width : width / 2;
            int h = (i == 0) ? height : height / 2;
            for (int row = 0; row < h; row++) {
                int bytesPerPixel = ImageFormat.getBitsPerPixel(ImageFormat.YUV_420_888) / 8;
                if (pixelStride == bytesPerPixel) {
                    int length = w * bytesPerPixel;
                    buffer.get(data, offset, length);

                    // Advance buffer the remainder of the row stride, unless on the last row.
                    // Otherwise, this will throw an IllegalArgumentException because the buffer
                    // doesn't include the last padding.
                    if (h - row != 1) {
                        buffer.position(buffer.position() + rowStride - length);
                    }
                    offset += length;
                } else {
                    // On the last row only read the width of the image minus the pixel stride
                    // plus one. Otherwise, this will throw a BufferUnderflowException because the
                    // buffer doesn't include the last padding.
                    if (h - row == 1) {
                        buffer.get(rowData, 0, width - pixelStride + 1);
                    } else {
                        buffer.get(rowData, 0, rowStride);
                    }

                    for (int col = 0; col < w; col++) {
                        data[offset++] = rowData[col * pixelStride];
                    }
                }
            }
        }

        return data;
    }

    @Override
    protected String doInBackground(Void... voids) {
        byte[] bytes = imagetoBytes(mImage);
        mImage.close();
        String timeStamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
        RequestBody requestBody = new MultipartBody.Builder()
                .setType(MultipartBody.FORM)
                .addFormDataPart("file", timeStamp + ".jpg", RequestBody.create(MEDIA_TYPE_JPEG, bytes))
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
