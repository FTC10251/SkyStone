package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.State;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

public class VuforiaLocalizerImplSubclass extends VuforiaLocalizerImpl {
    boolean updated = false;
    public Image rgb;
    class ClosableFrame extends Frame {
        public ClosableFrame(Frame other) {
            super(other);
        }
        public void close() {
            super.delete();
        }
    }

    public class VuforiaCallbackSubclass extends VuforiaLocalizerImpl.VuforiaCallback {
        @Override public synchronized void Vuforia_onUpdate(State state) {
            updated = true;
            super.Vuforia_onUpdate(state);
            CloseableFrame frame = new CloseableFrame(state.getFrame());
            RobotLog.vv(TAG, "Recieved Vuforia Frame Data", frame.getIndex());

            long num = frame.getNumImages();
            for(int i = 0; i < num; i++) {
                if(frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                    rgb = frame.getImage(i);
                }
            }
            frame.close();
        }
    }

    public VuforiaLocalizerImplSubclass(VuforiaLocalizer.Parameters parameters) {
        super(parameters);
        stopAR();
        clearGLSurface();
        this.vuforiaCallback = new VuforiaCallbackSubclass();
        stopAR();

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
    }

    public void clearGLSurface() {
        if(this.glSurfaceParent != null) {
            appUtil.synchronousRunOnUiThread(new Runnable() {
                @Override public void run() {
                    glSurfaceParent.removeAllViews();
                    glSurfaceParent.getOverlay().clear();
                    glSurface = null;
                }
            });
        }
    }
}
