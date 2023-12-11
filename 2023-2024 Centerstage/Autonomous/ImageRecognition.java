package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.os.SystemClock;
import android.util.Log;

import java.io.File;
import java.io.IOException;
import java.util.List;
import org.tensorflow.lite.support.image.ImageProcessor;
import org.tensorflow.lite.support.image.TensorImage;
import org.tensorflow.lite.support.image.ops.Rot90Op;
import org.tensorflow.lite.task.core.BaseOptions;
import org.tensorflow.lite.task.vision.classifier.Classifications;
import org.tensorflow.lite.task.vision.classifier.ImageClassifier;
import android.graphics.BitmapFactory;


public class ImageRecognition
{
    private static final String TAG = "Classifier";
    private static final int DELEGATE_CPU = 0;
    private static final int DELEGATE_NNAPI = 2;
    private static final int MODEL_MOBILENETV1 = 0;
    private static final int MODEL_EFFICIENTNETV0 = 1;
    private static final int MODEL_EFFICIENTNETV1 = 2;
    private static final int MODEL_EFFICIENTNETV2 = 3;

    private float threshold;
    private int numThreads;
    private int maxResults;
    private int currentDelegate;
    private int currentModel;
    private long inferenceTime;
    private ImageClassifier imageClassifier;
    private BitmapFactory bmpConverter;

    public ImageRecognition(Float threshold,
                            int numThreads,
                            int maxResults,
                            int currentDelegate,
                            int currentModel) {
        this.threshold = threshold;
        this.numThreads = numThreads;
        this.maxResults = maxResults;
        this.currentDelegate = currentDelegate;
        this.currentModel = currentModel;
        setupImageClassifier();
        bmpConverter = new BitmapFactory();
    }

    public static ImageRecognition create() {
        return new ImageRecognition(
                0.5f,
                2,
                3,
                0,
                0
        );
    }

    public float getThreshold() {
        return threshold;
    }

    public void setThreshold(float threshold) {
        this.threshold = threshold;
    }

    public int getNumThreads() {
        return numThreads;
    }

    public void setNumThreads(int numThreads) {
        this.numThreads = numThreads;
    }

    public int getMaxResults() {
        return maxResults;
    }

    public void setMaxResults(int maxResults) {
        this.maxResults = maxResults;
    }

    public void setCurrentDelegate(int currentDelegate) {
        this.currentDelegate = currentDelegate;
    }

    public void setCurrentModel(int currentModel) {
        this.currentModel = currentModel;
    }

    private void setupImageClassifier() {
        //List<String> labelList = new ArrayList<String>();
        //labelList.add("Box");
        //labelList.add("Not Box");
        ImageClassifier.ImageClassifierOptions.Builder optionsBuilder =
                ImageClassifier.ImageClassifierOptions.builder()
                        .setScoreThreshold(threshold)
                        .setMaxResults(maxResults);
                        //.setLabelAllowList(labelList);

        BaseOptions.Builder baseOptionsBuilder =
                BaseOptions.builder().setNumThreads(numThreads);

        /*switch (currentDelegate) {
            case DELEGATE_CPU:
                // Default
                break;
            case DELEGATE_NNAPI:
                baseOptionsBuilder.useNnapi();
        }*/

        String modelName = "model.tflite";
        /*switch (currentModel) {
            case MODEL_MOBILENETV1:
                modelName = "model.tflite";
                break;
            case MODEL_EFFICIENTNETV0:
                modelName = "model.tflite";
                break;
            case MODEL_EFFICIENTNETV1:
                modelName = "model.tflite";
                break;
            case MODEL_EFFICIENTNETV2:
                modelName = "model.tflite";
                break;
            default:
                modelName = "model.tflite";
        }*/
        try {
            imageClassifier =
                    ImageClassifier.createFromFileAndOptions(
                            new File("/sdcard/FIRST/tflitemodels/model.tflite"),
                            optionsBuilder.build());
        } catch (IOException e) {
            Log.e(TAG, "TFLite failed to load model with error: "
                    + e.getMessage());
        }
    }

    public List<Classifications> classify(Bitmap image, int imageRotation)
    {
        try
        {
            if (imageClassifier == null) {
                setupImageClassifier();
            }

            // Inference time is the difference between the system time at the start
            // and finish of the process
            long startTime = SystemClock.uptimeMillis();

            // Create preprocessor for the image.
            // See https://www.tensorflow.org/lite/inference_with_metadata/
            //            lite_support#imageprocessor_architecture
            ImageProcessor imageProcessor = new ImageProcessor.Builder().add(new Rot90Op(-imageRotation / 90)).build();

            // Preprocess the image and convert it into a TensorImage for classification.
            TensorImage tensorImage = imageProcessor.process(TensorImage.fromBitmap(image));

            inferenceTime = SystemClock.uptimeMillis() - startTime;
            if (image != null) {return imageClassifier.classify(tensorImage);}
            else {return null;}
        }
        catch (Exception e)
        {
            return null;
        }
    }

    public Bitmap PNG2BMP(File png) {return BitmapFactory.decodeFile(png.getAbsolutePath());}
    public long getInferenceTime() {return inferenceTime;}

    public void clearImageClassifier() {
        imageClassifier = null;
    }
}
