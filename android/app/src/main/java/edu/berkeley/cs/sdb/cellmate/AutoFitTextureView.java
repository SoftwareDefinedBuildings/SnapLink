package edu.berkeley.cs.sdb.cellmate;

import android.content.Context;
import android.util.AttributeSet;
import android.view.TextureView;

/**
 * A TextureView that can be adjusted to a specified aspect ratio.
 */
public class AutoFitTextureView extends TextureView {

    private int mPreviewWidth = 0;
    private int mPreviewHeight = 0;
    private int mSurfaceWidth = 0;
    private int mSurfaceHeight = 0;
    private float mTranslationX = 0.0f;
    private float mTranslationY = 0.0f;

    public AutoFitTextureView(Context context) {
        this(context, null);
    }

    public AutoFitTextureView(Context context, AttributeSet attrs) {
        this(context, attrs, 0);
    }

    public AutoFitTextureView(Context context, AttributeSet attrs, int defStyle) {
        super(context, attrs, defStyle);
    }

    /**
     * Sets the aspect ratio for this view.
     *
     * @param previewWidth  horizontal size of the preview
     * @param previewHeight vertical size of the preview
     */
    public void setAspectRatio(int previewWidth, int previewHeight) {
        if (previewWidth < 0 || previewHeight < 0) {
            throw new IllegalArgumentException("Size cannot be negative.");
        }
        if (mPreviewWidth == previewWidth && mPreviewHeight == previewHeight) {
            return;
        }

        if (previewWidth == 0 || previewHeight == 0) {
            return;
        }

        mPreviewWidth = previewWidth;
        mPreviewHeight = previewHeight;

        mSurfaceWidth = getMeasuredWidth();
        mSurfaceHeight = getMeasuredHeight();

        if (mSurfaceHeight < mSurfaceWidth * mPreviewHeight / mPreviewWidth) {
            int newSurfaceHeight = mSurfaceWidth * mPreviewHeight / mPreviewWidth;
            mTranslationY = (mSurfaceHeight - newSurfaceHeight) / 2;
            mSurfaceHeight = newSurfaceHeight;
        } else {
            int newSurfaceWidth = mSurfaceHeight * mPreviewWidth / mPreviewHeight;
            mTranslationX = (mSurfaceWidth - newSurfaceWidth) / 2;
            mSurfaceWidth = newSurfaceWidth;
        }

        requestLayout();
    }

    @Override
    protected void onMeasure(int widthMeasureSpec, int heightMeasureSpec) {
        super.onMeasure(widthMeasureSpec, heightMeasureSpec);

        int width = MeasureSpec.getSize(widthMeasureSpec);
        int height = MeasureSpec.getSize(heightMeasureSpec);

        if (mSurfaceWidth == 0 || mSurfaceHeight == 0) {
            mSurfaceWidth = width;
            mSurfaceHeight = height;
        }

        setMeasuredDimension(mSurfaceWidth, mSurfaceHeight);
        setTranslationX(mTranslationX);
        setTranslationY(mTranslationY);
    }

}