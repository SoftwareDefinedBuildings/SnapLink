package edu.berkeley.cs.sdb.SDBVision;

import android.os.AsyncTask;

import java.io.IOException;
import java.util.concurrent.Semaphore;

import edu.berkeley.cs.sdb.bosswave.BosswaveClient;
import edu.berkeley.cs.sdb.bosswave.ChainElaborationLevel;
import edu.berkeley.cs.sdb.bosswave.PayloadObject;
import edu.berkeley.cs.sdb.bosswave.PublishRequest;
import edu.berkeley.cs.sdb.bosswave.Response;
import edu.berkeley.cs.sdb.bosswave.ResponseHandler;

public class BosswavePublishTask extends AsyncTask<Void, Void, String> {
    private BosswaveClient mBosswaveClient;
    private String mTopic;
    private byte[] mData;
    private Listener mTaskListener;
    private Semaphore mSem;
    private String mResult;

    public interface Listener {
        void onResponse(String response);
    }

    public BosswavePublishTask(BosswaveClient bosswaveClient, String topic, byte[] data, Listener listener) {
        mBosswaveClient = bosswaveClient;
        mTopic = topic;
        mData = data;
        mTaskListener = listener;
        mSem = new Semaphore(0);
    }

    private ResponseHandler mResponseHandler = new ResponseHandler() {
        @Override
        public void onResponseReceived(Response response) {
            mResult = response.getStatus();
            mSem.release();
        }
    };

    @Override
    protected String doInBackground(Void... voids) {
        try {
            PublishRequest.Builder builder = new PublishRequest.Builder(mTopic);
            builder.setAutoChain(true);
            builder.setChainElaborationLevel(ChainElaborationLevel.FULL);
            builder.clearPayloadObjects();
            PayloadObject.Type msgPackType = new PayloadObject.Type(new byte[]{2, 0, 5, 1}); // this is hard coded because we know it's a msgpack
            byte[] poContents = mData;
            PayloadObject po = new PayloadObject(msgPackType, poContents);
            builder.addPayloadObject(po);
            PublishRequest request = builder.build();

            mBosswaveClient.publish(request, mResponseHandler);
        } catch (IOException e) {
            e.printStackTrace();
        }

        try {
            mSem.acquire();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        return mResult;
    }

    @Override
    protected void onPostExecute(String response) {
        mTaskListener.onResponse(response);
    }
}
