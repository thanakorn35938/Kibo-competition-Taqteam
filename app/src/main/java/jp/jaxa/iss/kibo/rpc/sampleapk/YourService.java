package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.graphics.Bitmap;
import android.util.Log;
import android.view.ViewDebug;

import net.sourceforge.zbar.Config;
import net.sourceforge.zbar.Image;
import net.sourceforge.zbar.ImageScanner;
import net.sourceforge.zbar.Symbol;
import net.sourceforge.zbar.SymbolSet;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

public class YourService extends KiboRpcService {
    @Override
    protected void runPlan1() {
        api.judgeSendStart();
        String p1_1_con[] = new String[2];
        String p1_2_con[] = new String[2];
        String p1_3_con[] = new String[2];
        String p2_1_con[] = new String[2];
        String p2_2_con[] = new String[2];
        String p2_3_con[] = new String[2];
        int arv = 0;

        String p1_1 = moveToQR(11.49, -5.7, 4.5, 0, 0, 0, 1,0);
        p1_1_con = p1_1.split(", ");
        double a = Double.parseDouble(p1_1_con[1]);

        String p1_3 = moveToQR(11, -5.5, 4.36, 0.707, 0, -0.707, 0,2);
        p1_3_con = p1_3.split(", ");
        double b = Double.parseDouble(p1_3_con[1]);

        String p1_2 = moveToQR(11, -6, 5.44, 0.5, 0.5, 0.5, -0.5,1);
        p1_2_con = p1_2.split(", ");
        double c = Double.parseDouble(p1_2_con[1]);

        moveToWrapper(10.50,-6.45,5.40,0,0,0,0);

        String p2_2 = moveToQR(11.49,-8,5,0,0,0,1,4);
        p2_2_con = p2_2.split(", ");
        double d = Double.parseDouble(p2_2_con[1]);

        String p2_3 = moveToQR(11,-7.7,5.44,0.7,0,0.7,0 ,5);
        p2_3_con = p2_3.split(", ");
        double e = Double.parseDouble(p2_3_con[1]);

        String p2_1 = moveToQR(10.41,-7.5,4.7,0,0,1,0,3);
        p2_1_con = p2_1.split(", ");
        double f = Double.parseDouble(p2_1_con[1]);
        //Ar PART

        moveToWrapper(10.95,-9.2,5.35,0,0,0,0);

        Mat ids = new Mat();
        while (arv == 0) {
            moveToWrapper(a, c, b, f, d, e, Math.sqrt(1 - (f * f) - (d * d) - (e * e)));
            Mat source = api.getMatNavCam();
            Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
            List<Mat> corners = new ArrayList<>();
            try {
                Aruco.detectMarkers(source, dictionary, corners, ids);
                arv = (int) ids.get(0, 0)[0];
            }
            catch (Exception ec) {
            }
        }
        api.judgeSendDiscoveredAR(Integer.toString(arv));
        api.laserControl(true);
        api.judgeSendFinishSimulation();
    }

    private void moveToWrapper(double pos_x, double pos_y, double pos_z,
                               double qua_x, double qua_y, double qua_z,
                               double qua_w)
    {

        final int LOOP_MAX = 3;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y,
                (float) qua_z, (float) qua_w);
        Result result = api.moveTo(point, quaternion, false);

        int loopCounter = 0;
        while (!result.hasSucceeded() || loopCounter < LOOP_MAX) {
            result = api.moveTo(point, quaternion, false);
            ++loopCounter;
        }
    }

    public String scanQRImage(Bitmap bMap) {
        String contents = null;

        int width = bMap.getWidth();
        int height = bMap.getHeight();
        int pixel[] = new int[width * height];
        bMap.getPixels(pixel, 0, width, 0, 0, width, height);
        Image barcode = new Image(width, height, "RGB4");


        barcode.setData(pixel);


        ImageScanner reader = new ImageScanner();
        reader.setConfig(Symbol.NONE, Config.ENABLE, 0);
        reader.setConfig(Symbol.QRCODE, Config.ENABLE, 1);


        Image barcode2 = barcode.convert("Y800");

        int result = reader.scanImage(barcode2);


        if (result != 0) {
            SymbolSet symbolSet = reader.getResults();
            for (Symbol symbol : symbolSet) {
                contents = symbol.getData();
            }
        }
        return contents;
    }

    public String moveToQR(double pos_x, double pos_y, double pos_z,
                           double qua_x, double qua_y, double qua_z,
                           double qua_w, int QR) {
        String back = null;
        while (back == null) {
            moveToWrapper(pos_x, pos_y, pos_z, qua_x, qua_y, qua_z, qua_w);
            Bitmap bmap = api.getBitmapNavCam();
            back = scanQRImage(bmap);
        }
        api.judgeSendDiscoveredQR(QR, back);
        Log.d("QR_Reading_Result", QR + "  " + back);
        return back;
    }
}