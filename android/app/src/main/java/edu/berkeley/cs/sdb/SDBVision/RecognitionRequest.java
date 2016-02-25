package edu.berkeley.cs.sdb.SDBVision;

import android.media.Image;

import com.android.volley.NetworkResponse;
import com.android.volley.ParseError;
import com.android.volley.Request;
import com.android.volley.Response;
import com.android.volley.toolbox.HttpHeaderParser;

import java.io.UnsupportedEncodingException;
import java.nio.ByteBuffer;

public class RecognitionRequest extends Request<String> {
    public static final String CONTENT_TYPE = "image/jpeg";

    private final Image mImage;
    private final Response.Listener<String> mListener;

    public RecognitionRequest(String url, Image image, Response.Listener<String> listener, Response.ErrorListener errorListener) {
        super(Method.POST, url, errorListener);
        this.mImage = image;
        this.mListener = listener;
    }

    @Override
    public String getBodyContentType() {
        return CONTENT_TYPE;
    }

    @Override
    public byte[] getBody() {
        ByteBuffer buffer = mImage.getPlanes()[0].getBuffer();
        byte[] bytes = new byte[buffer.remaining()];
        buffer.get(bytes); // write buffer to bytes
        mImage.close();

        return bytes;
    }

    @Override
    protected void deliverResponse(String response) {
        mListener.onResponse(response);
    }

    @Override
    protected Response<String> parseNetworkResponse(NetworkResponse response) {
        try {
            String result = new String(response.data, HttpHeaderParser.parseCharset(response.headers));
            return Response.success(result, HttpHeaderParser.parseCacheHeaders(response));
        } catch (UnsupportedEncodingException e) {
            return Response.error(new ParseError(e));
        }
    }
}
