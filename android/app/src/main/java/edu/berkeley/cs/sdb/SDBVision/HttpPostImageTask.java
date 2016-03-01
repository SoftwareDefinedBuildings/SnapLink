package edu.berkeley.cs.sdb.SDBVision;

import android.media.Image;
import android.os.AsyncTask;

import org.apache.http.HttpEntity;
import org.apache.http.HttpResponse;
import org.apache.http.HttpStatus;
import org.apache.http.StatusLine;
import org.apache.http.client.HttpClient;
import org.apache.http.client.methods.HttpPost;
import org.apache.http.entity.ContentType;
import org.apache.http.entity.mime.MultipartEntityBuilder;
import org.apache.http.protocol.BasicHttpContext;
import org.apache.http.protocol.HttpContext;
import org.apache.http.util.EntityUtils;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.text.SimpleDateFormat;
import java.util.Date;

public class HttpPostImageTask extends AsyncTask<Void, Void, String> {
    private HttpClient mHttpClient;
    private String mUrl;
    private Image mImage;
    private Listener mListener;


    public interface Listener {
        void onResponse(String response);
    }

    public HttpPostImageTask(HttpClient httpClient, String url, Image image, Listener listener) {
        this.mHttpClient = httpClient;
        this.mUrl = url;
        this.mImage = image;
        this.mListener = listener;
    }

    private void SaveImage(File file) {
        ByteBuffer buffer = mImage.getPlanes()[0].getBuffer();
        byte[] bytes = new byte[buffer.remaining()];
        buffer.get(bytes);
        FileOutputStream output = null;
        try {
            output = new FileOutputStream(file);
            output.write(bytes);
        } catch (IOException e) {
            e.printStackTrace();
        } finally {
            mImage.close();
            if (output != null) {
                try {
                    output.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    @Override
    protected String doInBackground(Void... voids) {
        MultipartEntityBuilder builder = MultipartEntityBuilder.create();
        ByteBuffer buffer = mImage.getPlanes()[0].getBuffer();
        byte[] bytes = new byte[buffer.remaining()];
        buffer.get(bytes); // write buffer to bytes
        mImage.close();
        String timeStamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
        builder.addBinaryBody("file", bytes, ContentType.create("image/jpeg"), timeStamp + ".jpg");
        HttpEntity httpEntity = builder.build();

        HttpContext localContext = new BasicHttpContext();
        HttpPost httpPost = new HttpPost(mUrl);
        httpPost.setEntity(httpEntity);

        String result = null;
        try {
            HttpResponse response = mHttpClient.execute(httpPost, localContext);
            StatusLine statusLine = response.getStatusLine();
            if (statusLine.getStatusCode() != HttpStatus.SC_OK) {
                String msg = String.format("HTTP Error %d: %s", statusLine.getStatusCode(), statusLine.getReasonPhrase());
                throw new RuntimeException(msg);
            }
            result = EntityUtils.toString(response.getEntity());
        } catch (IOException e) {
            e.printStackTrace();
        }

        if (result != null && !result.trim().equals("None")) {
            return result.trim();
        }

        return null;
    }

    @Override
    protected void onPostExecute(String response) {
        mListener.onResponse(response);
    }
}
